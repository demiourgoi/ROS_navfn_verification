# -*- coding: utf-8 -*-

import argparse
import time
import maude
import math
import json
import os.path


class DirectProfiler:
    """ Profile the Maude A* algorithm directly """

    ASTAR_MAUDE_PATH = '../maude/astar_no_turnNavFnPlanner.maude'

    def __init__(self, test_path, obtain_navfn=True):
        with open(test_path, 'r') as ftest:
            lines = ftest.readlines()
            self.w, self.h = lines[0].strip().split()
            self.w = int(self.w)
            self.h = int(self.h)
            self.map_bin_file = lines[1].strip()  # Filename relative to the test case
            self.map_data = [0] * (self.w * self.h)  # Avoid appending
            test_full_path = os.path.abspath(ftest.name)
            test_dir = os.path.dirname(test_full_path)
            map_full_map_path = os.path.join(test_dir, self.map_bin_file)
            self.map_bin_file = map_full_map_path  # Full path

            with open(self.map_bin_file, 'rb') as fmap:
                for i in range(self.w * self.h):
                    cell = fmap.read(1)
                    self.map_data[i] = int.from_bytes(cell, 'big')  # It's one byte, it doesn't matter big or little endian
                
            self.test_cases = list()
            for test in lines[3:]:
                if test.strip().startswith('-1'):
                    continue
                x0, y0, x1, y1 = [float(c) for c in test.strip().split()]
                self.test_cases.append(((x0, y0, 0.0), (x1, y1, 0.0)))  # Orientation 0 degrees

        maude.init()
        maude.load(self.ASTAR_MAUDE_PATH)

        # Find sorts and operators needed to construct the a* term
        self.m       = maude.getModule('ASTAR')
        pose_kind    = self.m.findSort('Pose').kind()
        costmap_kind = self.m.findSort('CostMap').kind()
        float_kind   = self.m.findSort('Float').kind()
        path_kind    = self.m.findSort('Path').kind()
        int_kind     = self.m.findSort('IntList').kind()
        nat_kind     = self.m.findSort('Nat').kind()
        potential_kind = self.m.findSort('Potential').kind()
        gradient_kind = self.m.findSort('Gradient').kind()

        # We need it to solve an ambiguity later
        self.pose_kind = pose_kind

        # Use different functions whether obtaining the potential or not
        if not obtain_navfn:
            self.astar = self.m.findSymbol('a*', [pose_kind, pose_kind, costmap_kind,
                                             nat_kind, nat_kind, nat_kind], 
                                             path_kind)
            self.get_potential = None
            self.compute_path = None

        else:
            self.astar = None

            # op getPotential : Pose Pose CostMap Nat Nat Nat -> Potential .
            self.get_potential = self.m.findSymbol('getPotential',
                                                   [pose_kind, pose_kind, costmap_kind, nat_kind, nat_kind, nat_kind],
                                                   potential_kind)

            # op computePath : CostMap Potential Pose Pose Float Gradient Nat Nat Nat -> Path .
            self.compute_path = self.m.findSymbol('computePath',
                                                  [potential_kind, pose_kind, pose_kind, float_kind, gradient_kind, nat_kind, nat_kind, nat_kind],
                                                  path_kind)

        intlist      = self.m.findSymbol('_`,_', [int_kind] * 2, int_kind) 
        cmap         = self.m.findSymbol('`{_`}', [int_kind], costmap_kind)

        self.pattern = self.m.parseTerm('{ X:Float, Y:Float, Z:Float } O:Nat')

        self.mod = self.m

        # Constants that will be used multiple times
        zero = self.m.parseTerm('0')
        one  = self.m.parseTerm('1')
        mtIL = self.m.parseTerm('mtIL')
        self.noPath = self.m.parseTerm('noPath')
        self.empyList = self.m.parseTerm('nil')
        
        # Dictionary with Maude integer terms from 0 to 255 to avoid parsing 
        # when constructing the map_list in Maude
        int_terms = dict()
        for i in range(0,256):
            int_terms[i] = self.m.parseTerm(str(i))

        # Build the IntList with the costmap data
        map_list = mtIL

        for c in self.map_data:
            map_list = intlist(map_list, int_terms[c])

        cycles = str(max(int(self.w * self.h / 20), self.w + self.h))  # Same number of cycles as ROS
        path_cycles = str(4 * self.w)
        self.static_args = [
            cmap(map_list),
            self.m.parseTerm(str(int(self.h))),
            self.m.parseTerm(str(int(self.w))),
            self.m.parseTerm(cycles),
            self.m.parseTerm(path_cycles),
        ]

        # Hook for "op get : CostMap Nat Nat Nat -> Float"
        class MapHook(maude.Hook):
            def __init__(self, parent):
                super().__init__()
                self.parent = parent  # DirectProfiler object to access attributes as the map or the Maude module for parsing
                self.cache = dict()  # Dictionary int in [0..255] -> Maude term representing the corresponding Float
                for i in range(0, 256):
                    self.cache[i] = self.parent.m.parseTerm(str(float(i)))
            
            def run(self, term, data):
                try:
                    _, x, y, ncols = [int(arg) for arg in term.arguments()]
                    cell_value = self.parent.map_data[x + y * ncols]
                    ret_term = self.cache[cell_value]
                    # print(f'FAST {term} --> {ret_term}')
                    return ret_term
                except Exception as e:
                    print(e)

        self.mapHook = MapHook(self)
        maude.connectEqHook('get', self.mapHook)

    def run_test_suite(self):          
        for origin, dest in self.test_cases:
            print(origin, dest)
            self.run_astar(origin, dest)

    def show_result(self, initial, goal, duration, length, term, potarr):
        line = dict()
        line['initial'] = [self.int_float(x) for x in initial]
        line['goal'] = [self.int_float(x) for x in goal]
        line['duration'] = duration
        line['length'] = int(length) if length % 1 == 0.0 else float(f"{length:.5f}")

        path = list()
        # Iterable of poses, handles unitary paths
        if term == self.noPath:
            poses = []
        elif term.getSort() == self.m.findSort('Pose'):
            poses = [term] 
        else: 
            poses = term.arguments()
            
        for pose in poses:
            x, y, t = self.destruct_pose(pose)
            path.append([self.int_float(x), self.int_float(y)])
        line['path'] = path
        if potarr is not None:
            line['navfn'] = [float(entry) for row in potarr.arguments() for entry in row.arguments()]
        print(json.dumps(line))

    def run_astar(self, origin, dest):
        x0, y0, t0 = origin
        x,  y,  t  = dest

        # Build the a* or computePath term with makeTerm
        if self.astar is not None:
            # The a* function is directly call because we are not interested in the potential
            term = self.astar(
                    self.mod.parseTerm('{{{}, {}, 0.0}} {}'.format(float(x0), float(y0), int(t0))),
                    self.mod.parseTerm('{{{}, {}, 0.0}} {}'.format(float(x), float(y), int(t))),
                    *self.static_args
            )

            potarr = None
        else:
            # The calculation is decomposed in getPotential and computePath, so that we can obtain
            # the potential calculated by Maude
            potarr = self.get_potential(
                self.mod.parseTerm('{{{}, {}, 0.0}} {}'.format(float(x0), float(y0), int(t0))),
                self.mod.parseTerm('{{{}, {}, 0.0}} {}'.format(float(x), float(y), int(t))),
                *self.static_args[:-1]
            )

            # Only the computePath term will be reduced, but the subterm potarr
            # will get reduced by the way
            term = self.compute_path(
                potarr,
                self.mod.parseTerm(f'{{{int(x0)}, {int(y0)}}}', self.pose_kind),
                self.mod.parseTerm(f'{{{int(x)}, {int(y)}}}', self.pose_kind),
                self.mod.parseTerm('stepSize'),
                self.mod.parseTerm(f'initialGradient({self.h}, {self.w})'),
                *self.static_args[-4:-2],
                self.static_args[-1],
            )

        start_time = time.perf_counter()
        call = term.copy()
        term.reduce()
        end_time = time.perf_counter()

        if not term.getSort() <= self.m.findSort('Path'):
            print(f'ERROR when reducing {call} -> {term}')
            return
        print(f'OK when reducing {call} -> {term}')

        hmtime = end_time - start_time
        length, rotation, numrot = self.calculate_length(term)
        self.show_result([x0, y0], [x, y], hmtime, length, term, potarr)

    def int_float(self, x):
        return int(x) if x % 1 == 0.0 else x

    def destruct_pose(self, pose):
        """ Obtain the values of a Maude pose """
        # Match the pose into the pattern (we assume there is always a single match)
        it = pose.match(self.pattern)
        subs = next(it)
        return float(str(subs.find('X'))), float(str(subs.find('Y'))), int(str(subs.find('O')))

    def calculate_length(self, mresult):
        """ Calculate the length, total rotation and number of rotations of a path """

        if mresult == self.noPath or mresult.getSort() == self.m.findSort('Pose'):
            return 0, 0, 0	          

        s  = 0.0     # Length
        st = 0.0     # Total rotation
        nr = 0       # Number of rotations

        it    = mresult.arguments()
        first = next(it)

        x, y, t = self.destruct_pose(first)

        for pose in it:
            if str(pose.symbol()) == 'noPath':
                break

            cx, cy, ct = self.destruct_pose(pose)

            if t != ct:
                nr += 1

            s  += math.sqrt((cx - x) ** 2 + (cy - y) ** 2)
            st += abs(ct - t)
            x, y, t = cx, cy, ct

        return s, st, nr


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Test the Maude implementation of the A* algorithm')
    parser.add_argument('test_file', help='File specifying the test cases (using the syntax of profile_cpp)')
    parser.add_argument('--no-navfn', dest='navfn', action='store_false', help='Do not obtain the navigation function')

    args = parser.parse_args()

    dprofiler = DirectProfiler(args.test_file, args.navfn)
    dprofiler.run_test_suite()

