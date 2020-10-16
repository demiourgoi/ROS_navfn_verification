# -*- coding: utf-8 -*-

import time, csv, maude, math

import default_costmap

TEST_SUITE = [
    [(161, 194, 0.0), (162, 194, 0.0)],
    [(161, 194, 0.0), (161, 193, 0.0)],
    [(161, 194, 0.0), (162, 193, 0.0)],
    [(161, 194, 0.0), (168, 194, 0.0)],
    [(162, 189, 45.0), (236, 189, 180.0)],
    [(162, 189, 45.0), (236, 189, 180.0)],
    # (−1.9, -0.55) -> (1.75, 0.5)
    [(162, 189, 0.0), (235, 210, 0.0)],
    [(162, 189, 0.0), (212, 237, 20.0)],
    [(162, 189, 0.0), (287, 237, 30.0)],
    [(162, 189, 0.0), (163, 211, 40.0)],
    [(162, 189, 0.0), (199, 161, 50.0)],
    [(162, 189, 0.0), (235, 198, 60.0)],
    [(190, 222, 180.0), (211, 222, 0.0)],
    [(189, 211, 135.0), (211, 189, 45.0)],
    # Not possible
    # [(156, 200, 0.0), (178, 200, 0.0)],
]

class DirectProfiler:
    '''Profile the Maude A* algorithm directly'''

    ASTAR_MAUDE_PATH = '../maude/astar.maude'

    def __init__(self):
        maude.init()
        maude.load(self.ASTAR_MAUDE_PATH)

        # Find sorts and operators needed to construct the a* term
        m            = maude.getModule('ASTAR')
        pose_kind    = m.findSort('Pose').kind()
        costmap_kind = m.findSort('CostMap').kind()
        float_kind   = m.findSort('Float').kind()
        path_kind    = m.findSort('Path').kind()
        int_kind     = m.findSort('IntList').kind()

        self.astar   = m.findSymbol('a*', [pose_kind, pose_kind, costmap_kind, float_kind, float_kind], path_kind)
        intlist      = m.findSymbol('_`,_', [int_kind] * 2, int_kind) 
        cmap         = m.findSymbol('`{_`}', [int_kind], costmap_kind)

        self.pattern = m.parseTerm('{ X:Float, Y:Float, Z:Float } O:Nat')

        self.mod = m

        # Constants that will be used multiple times
        zero = m.parseTerm('0')
        one  = m.parseTerm('1')
        mtIL = m.parseTerm('mtIL')

        # Build the IntList with the costmap data
        map_list = mtIL

        for c in default_costmap.data:
                if c < 50:
                        map_list = intlist(map_list, zero)
                else:
                        map_list = intlist(map_list, one)

        self.static_args = [
            cmap(map_list),
            m.parseTerm(str(float(default_costmap.height))),
            m.parseTerm(str(float(default_costmap.width)))
        ]

        class MapHook(maude.Hook):
            def run(self, term, data):
                x, y, ncols = [int(arg) for arg in term.arguments()]
                return data.getTerm('trueTerm' if default_costmap.data[x + y * ncols] < 50 else 'falseTerm')

        self.mapHook = MapHook()
        maude.connectEqHook('open2?', self.mapHook)

        # CSV where results are saved
        self.csvfile = open('resultsd.csv', 'w')
        self.csvwriter = csv.writer(self.csvfile)

    def run_test_suite(self):
        for origin, dest in TEST_SUITE:
            self.run_astar(origin, dest)

    def run_astar(self, origin, dest):
        x0, y0, t0 = origin
        x,  y,  t  = dest

        # Build the a* term with makeTerm
        term = self.astar(
                self.mod.parseTerm('{{{}, {}, 0.0}} {}'.format(float(x0), float(y0), int(t0))),
                self.mod.parseTerm('{{{}, {}, 0.0}} {}'.format(float(x), float(y), int(t))),
                *self.static_args
        )
        maude.input('do clear memo .')
        start_time = time.perf_counter()
        term.reduce()
        end_time = time.perf_counter()

        hmtime = end_time - start_time
        length, rotation, numrot = self.calculate_length(term)

        print('Time:', hmtime)
        print('Length:', length)
        print('Acc. rotation:', rotation)
        print('# rotations:', numrot)
        self.csvwriter.writerow([False] + list(origin) + list(dest) + [hmtime, length, rotation, numrot])

    def destruct_pose(self, pose):
        '''Obtain the values of a Maude pose'''
        # Match the pose into the pattern (we assume there is always a single match)
        subs = next(pose.match(self.pattern))

        return float(str(subs.find('X'))), float(str(subs.find('Y'))), int(str(subs.find('O')))

    def calculate_length(self, mresult):
        '''Calculate the length, total rotation and number of rotations of a path'''

        if str(mresult.symbol()) == 'noPath':
            return 0.0

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
    dprofiler = DirectProfiler()
    dprofiler.run_test_suite()
