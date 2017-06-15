import unittest
import numpy as np

import navigatable_area as nav


class TestStringMethods(unittest.TestCase):

    def test_area_connection(self):
        li = nav.L_IMPACT
        ri = nav.R_IMPACT
        lfc = nav.L_FRONT_CLOSE
        lff = nav.L_FRONT_FAR
        rfc = nav.R_FRONT_CLOSE
        rff = nav.R_FRONT_FAR
        lec = nav.L_EDGE_CLOSE
        lef = nav.L_EDGE_FAR
        rec = nav.R_EDGE_CLOSE
        ref = nav.R_EDGE_FAR
        lg = nav.L_GUIDE
        rg = nav.R_GUIDE

        self.assertTrue(li.is_left_to(ri), "Left Impact to Right Impact")
        self.assertTrue(li.is_top_edge_align(ri), "Left Impact to Right Impact")
        self.assertTrue(li.is_bottom_edge_align(ri), "Left Impact to Right Impact")

        self.assertTrue(lfc.is_top_to(li), "Left Front to Left Impact")
        self.assertTrue(lfc.is_left_edge_align(li), "Left Front to Left Impact")
        self.assertTrue(lfc.is_right_edge_align(li), "Left Front to Left Impact")

        self.assertTrue(lfc.is_bottom_to(lff), "Left Front Close to Left Front Far")
        self.assertTrue(lfc.is_left_edge_align(lff), "Left Front Close to Left Front Far")
        self.assertTrue(lfc.is_right_edge_align(lff), "Left Front Close to Left Front Far")

        self.assertTrue(lfc.is_left_to(rfc), "Left Front Close to Right Front Close")
        self.assertTrue(lfc.is_top_edge_align(rfc), "Left Front Close to Right Front Close")
        self.assertTrue(lfc.is_bottom_edge_align(rfc), "Left Front Close to Right Front Close")

        self.assertTrue(lff.is_top_edge_align(rff), "Left Front Far to Right Front Far")

        self.assertTrue(rfc.is_top_to(ri), "Right Front to Right Impact")
        self.assertTrue(rfc.is_left_edge_align(ri), "Right Front to Right Impact")
        self.assertTrue(rfc.is_right_edge_align(ri), "Right Front to Right Impact")

        self.assertTrue(rfc.is_bottom_to(rff), "Right Front Close to Right Front Far")
        self.assertTrue(rfc.is_left_edge_align(rff), "Right Front Close to Right Front Far")
        self.assertTrue(rfc.is_right_edge_align(rff), "Right Front Close to Right Front Far")

        self.assertTrue(lec.is_left_to(li), "Left Edge Close to Left Impact")

        self.assertTrue(lec.is_left_edge_align(lef), "Left Edge Close to Left Edge Far")
        self.assertTrue(lec.is_right_edge_align(lef), "Left Edge Close to Left Edge Far")
        self.assertTrue(lec.is_bottom_to(lef), "Left Edge Close to Left Edge Far")

        self.assertTrue(rec.is_right_to(ri), "Right Edge Close to Right Impact")

        self.assertTrue(rec.is_left_edge_align(ref), "Right Edge Close to Right Edge Far")
        self.assertTrue(rec.is_right_edge_align(ref), "Right Edge Close to Right Edge Far")
        self.assertTrue(rec.is_bottom_to(ref), "Right Edge Close to Right Edge Far")

        self.assertTrue(lg.is_left_to(lec), "Left Guide to Left Edge")

        self.assertTrue(rg.is_right_to(rec), "Right Guide to Right Edge")

    def test_render_img(self):
        img1 = np.zeros((200, 10))
        img2 = np.zeros((10, 300))
        self.assertEqual(-1, nav.render_image(img1, nav.L_IMPACT, 0), "Invalid shape check")
        self.assertEqual(-1, nav.render_image(img2, nav.L_IMPACT, 0), "Invalid shape check")
        img = np.zeros((160, 320))
        nav.render_image(img, nav.L_IMPACT, 255)
        li = nav.L_IMPACT
        self.assertEqual(255, img[li.y1 + 1, li.x1 + 1], "Change in shape")
        self.assertEqual(0, img[li.y1 - 1, li.x1 - 1], "Not change out of shape")

        nav.render_image(img, nav.L_IMPACT, 255)
        self.assertEqual(255, img[li.y1 + 1, li.x1 + 1], "Verify clip")

        nav.render_image(img, 255, None, None)
        self.assertEqual(510, img[li.y1 + 1, li.x1 + 1], "Verify not clip")

    def test_navable_percent(self):
        li = nav.L_IMPACT
        img1 = np.zeros((200, 10))
        img2 = np.zeros((10, 300))
        self.assertEqual(-1, nav.render_image(img1, nav.L_IMPACT, 0), "Invalid shape")
        self.assertEqual(-1, nav.render_image(img2, nav.L_IMPACT, 0), "Invalid shape")

        img = np.zeros((160, 320))
        self.assertEqual(0, nav.navable_percent(img, nav.L_IMPACT), "0%")

        img = np.ones((160, 320))
        self.assertEqual(100, nav.navable_percent(img, nav.L_IMPACT), "100%")

        i = int((li.x2+1-li.x1)/2+li.x1)
        img[:, 0:i] = 0
        self.assertEqual(50, nav.navable_percent(img, nav.L_IMPACT), "50%")

    def test_navigatible_area_object(self):
        name = 'l_edge_close'
        area = nav.AREAS[name]

        img = np.zeros((160, 320))
        na = nav.NavigationArea(img)

        xs = range(area.x1, area.x2 + 1)
        ys = range(area.y1, area.y2 + 1)
        total = len(xs) * len(ys)

        count = 0
        for x in xs:
            for y in ys:
                img[y, x] = 1
                count += 1
                if count * 100 / total < nav.BLOCKED_THRESHOLD:
                    self.assertTrue(na.is_area_blocked(name))
                    self.assertFalse(na.is_area_normal(name))
                    self.assertFalse(na.is_area_open(name))
                    self.assertFalse(na.is_area_clear(name))
                if nav.OPEN_THRESHOLD > count * 100 / total > nav.BLOCKED_THRESHOLD:
                    self.assertFalse(na.is_area_blocked(name))
                    self.assertTrue(na.is_area_normal(name))
                    self.assertFalse(na.is_area_open(name))
                    self.assertFalse(na.is_area_clear(name))
                if nav.CLEAR_THRESHOLD > count * 100 / total > nav.OPEN_THRESHOLD:
                    self.assertFalse(na.is_area_blocked(name))
                    self.assertFalse(na.is_area_normal(name))
                    self.assertTrue(na.is_area_open(name))
                    self.assertFalse(na.is_area_clear(name))
                if count * 100 / total > nav.CLEAR_THRESHOLD:
                    self.assertFalse(na.is_area_blocked(name))
                    self.assertFalse(na.is_area_normal(name))
                    self.assertTrue(na.is_area_open(name))
                    self.assertTrue(na.is_area_clear(name))

if __name__ == '__main__':
    unittest.main()
