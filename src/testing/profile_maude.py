# -*- coding: utf-8 -*-

import argparse
import time
import maude
import math
import json
import os.path
import sys

from test_utils import load_test


class DirectProfiler:
    """ Profile the Maude A* algorithm directly """

    ASTAR_MAUDE_PATH = '../maude/astar_navfnplanner.maude'

    def __init__(self, test_path, obtain_navfn=True):
        self.w, self.h, self.map_data, self.test_cases, _ = load_test(test_path)

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
            self.astar = self.m.findSymbol('a*',
                                           [pose_kind, pose_kind, costmap_kind]
                                           + [nat_kind] * 4,
                                           path_kind)
            self.initialPotState = None
            self.initialPathState = None

        else:
            self.astar = None

            # op initialPotState : Pose Pose CostMap Nat Nat Nat -> Path .
            self.initialPotState = self.m.findSymbol('initialPotState',
                                                     (pose_kind, pose_kind, costmap_kind, nat_kind, nat_kind, nat_kind),
                                                     potential_kind)

            # op initialPathState : Pose Pose Nat Nat Nat Potential -> Path .
            self.initialPathState = self.m.findSymbol('initialPathState',
                                                      (pose_kind, pose_kind, nat_kind, nat_kind, nat_kind, potential_kind),
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
        # int_terms = dict()
        # for i in range(0,256):
        #    int_terms[i] = self.m.parseTerm(str(i))

        # Build the IntList with the costmap data
        map_list = mtIL

        # There is no need to build the map, because it will be read from the hook
        # for c in self.map_data:
        #    map_list = intlist(map_list, int_terms[c])

        cycles = str(max(int(self.w * self.h / 20), self.w + self.h))  # Same number of cycles as ROS
        path_cycles = str(4 * self.w)
        self.static_args = [
            cmap(map_list),
            self.m.parseTerm(str(int(self.w))),
            self.m.parseTerm(str(int(self.h))),
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
                    cell_value = self.parent.map_data[y * ncols + x]
                    ret_term = self.cache[cell_value]
                    # print(f'FAST {term} --> {ret_term}')
                    return ret_term
                except Exception as e:
                    print('hook:', e)

        self.mapHook = MapHook(self)
        maude.connectEqHook('get', self.mapHook)

    def run_test_suite(self, obtain_navf=True):
        if obtain_navf:
            print('Costmap:', self.map_data)
            print()
        for origin, dest in self.test_cases:
            print(origin, dest)
            self.run_astar(origin, dest)

    def show_result(self, initial, goal, duration, length, term, potarr):
        line = dict()
        line['initial'] = [self.int_float(x) for x in initial]
        line['goal'] = [self.int_float(x) for x in goal]
        # Omit duration for reproducibility
        # line['duration'] = duration
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
        if potarr:
            line['navfn'] = [float(entry) for row in potarr.arguments() for entry in row.arguments()]
        json.dump(line, sys.stdout)
        print()

    def run_astar(self, origin, dest):
        x0, y0, t0 = origin
        x,  y,  t  = dest

        # Build the a* or computePath term with makeTerm
        if self.astar:
            # The a* function is directly call because we are not interested in the potential
            term = self.astar(
                    self.mod.parseTerm('{{{}, {}, 0.0}} {}'.format(float(x0), float(y0), int(t0))),
                    self.mod.parseTerm('{{{}, {}, 0.0}} {}'.format(float(x), float(y), int(t))),
                    *self.static_args
            )

            potarr = None
        else:
            # The potential is calculated first by rewriting initialPotState
            term = self.initialPotState(
                self.mod.parseTerm('{{{}, {}, 0.0}} {}'.format(float(x0), float(y0), int(t0))),
                self.mod.parseTerm('{{{}, {}, 0.0}} {}'.format(float(x), float(y), int(t))),
                *self.static_args[:-1]
            )

        start_time = time.perf_counter()

        call = term.copy()
        term.rewrite()

        if self.astar is None:
            # term is the potential array except in case of failure
            potarr = term
            # Rewrting should continue to find the path
            term = self.initialPathState(
                self.mod.parseTerm(f'{{{int(x0)}, {int(y0)}}}', self.pose_kind),
                self.mod.parseTerm(f'{{{int(x)}, {int(y)}}}', self.pose_kind),
                *self.static_args[-4:-2],
                self.static_args[-1],
                potarr
            )
            term.rewrite()

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
        subs, _ = next(it)
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
    dprofiler.run_test_suite(args.navfn)
