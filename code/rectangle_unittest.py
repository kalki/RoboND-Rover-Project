import unittest
from rectangle import Rectangle


class MyTestCase(unittest.TestCase):

    def test_prod(self):
        x = Rectangle(10, 10, 19, 19)
        self.assertEqual(100, x.prod, "Prod")

    def test_position(self):
        r11 = Rectangle(10, 10, 19, 19)
        r12 = Rectangle(20, 10, 29, 19)
        r13 = Rectangle(30, 10, 39, 19)
        r21 = Rectangle(10, 20, 19, 29)
        r22 = Rectangle(20, 20, 29, 29)
        r31 = Rectangle(10, 30, 19, 39)

        self.assertTrue(r11.is_left_to(r12))
        self.assertTrue(r11.is_left_to(r22))
        self.assertFalse(r11.is_left_to(r13))
        self.assertFalse(r13.is_left_to(r11))

        self.assertTrue(r13.is_right_to(r12))
        self.assertTrue(r13.is_right_to(r22))
        self.assertFalse(r13.is_right_to(r11))
        self.assertFalse(r11.is_right_to(r13))

        self.assertTrue(r11.is_top_to(r21))
        self.assertTrue(r11.is_top_to(r22))
        self.assertFalse(r31.is_top_to(r11))
        self.assertFalse(r11.is_top_to(r31))

        self.assertTrue(r31.is_bottom_to(r21))
        self.assertTrue(r31.is_bottom_to(r22))
        self.assertFalse(r31.is_bottom_to(r11))
        self.assertFalse(r11.is_bottom_to(r31))

    def test_align(self):
        r11 = Rectangle(10, 10, 19, 19)
        r12 = Rectangle(20, 10, 29, 19)
        r21 = Rectangle(10, 20, 19, 29)
        r22 = Rectangle(20, 20, 29, 29)
        r23 = Rectangle(30, 20, 39, 29)
        r32 = Rectangle(20, 30, 29, 39)

        self.assertTrue(r12.is_left_edge_align(r22))
        self.assertTrue(r12.is_left_edge_align(r32))
        self.assertFalse(r12.is_left_edge_align(r11))

        self.assertTrue(r12.is_right_edge_align(r22))
        self.assertTrue(r12.is_right_edge_align(r32))
        self.assertFalse(r12.is_right_edge_align(r11))

        self.assertTrue(r21.is_top_edge_align(r22))
        self.assertTrue(r21.is_top_edge_align(r23))
        self.assertFalse(r21.is_top_edge_align(r11))

        self.assertTrue(r21.is_bottom_edge_align(r22))
        self.assertTrue(r21.is_bottom_edge_align(r23))
        self.assertFalse(r21.is_bottom_edge_align(r11))

if __name__ == '__main__':
    unittest.main()
