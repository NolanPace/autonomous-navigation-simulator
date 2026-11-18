import unittest
from navigation import Grid, Robot, parse_ascii, plan

class NavigationTests(unittest.TestCase):
    def test_shortest_path(self):
        g=Grid(5,5,{(2,0),(2,1),(2,2),(2,3)})
        a=plan(g,(0,0),(4,0));d=plan(g,(0,0),(4,0),'dijkstra')
        self.assertEqual(len(a),len(d));self.assertEqual(len(a)-1,12)
    def test_unreachable(self):
        self.assertEqual(plan(Grid(3,1,{(1,0)}),(0,0),(2,0)),[])
    def test_dynamic_obstacle(self):
        robot=Robot(Grid(3,3,set()),(0,0),(2,0));robot.add_obstacle((1,0))
        while robot.step(): pass
        self.assertEqual(robot.position,(2,0));self.assertNotIn((1,0),robot.travelled)
    def test_invalid_map(self):
        with self.assertRaises(ValueError):parse_ascii('S.\n.G#')
    def test_same_start_goal(self):
        self.assertEqual(plan(Grid(2,2,set()),(1,1),(1,1)),[(1,1)])
if __name__=='__main__':unittest.main()
