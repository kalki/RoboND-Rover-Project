class Rectangle(object):

    def __init__(self, x1, y1, x2, y2):
        self.x1 = x1
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2
        self.prod = (x2 + 1 - x1) * (y2 + 1 - y1)

    def __str__(self):
        return "x1:{:}, y1:{:} - x2:{:}, y2:{:}".format(self.x1, self.y1, self.x2, self.y2)

    def is_left_to(self, rect):
        if type(rect) == Rectangle:
            if self.x2 + 1 == rect.x1:
                return True
        return False

    def is_right_to(self, rect):
        if type(rect) == Rectangle:
            if self.x1 - 1 == rect.x2:
                return True
        return False

    def is_top_to(self, rect):
        if type(rect) == Rectangle:
            if self.y2 + 1 == rect.y1:
                return True
        return False

    def is_bottom_to(self, rect):
        if type(rect) == Rectangle:
            if self.y1 - 1 == rect.y2:
                return True
        return False

    def is_left_edge_align(self, rect):
        if type(rect) == Rectangle:
            if self.x1 == rect.x1:
                return True
        return False

    def is_right_edge_align(self, rect):
        if type(rect) == Rectangle:
            if self.x2 == rect.x2:
                return True
        return False

    def is_top_edge_align(self, rect):
        if type(rect) == Rectangle:
            if self.y1 == rect.y1:
                return True
        return False

    def is_bottom_edge_align(self, rect):
        if type(rect) == Rectangle:
            if self.y2 == rect.y2:
                return True
        return False

    def is_contained(self, shape):
        if shape[0] <= self.y2 or shape[1] <= self.x2:
            return False
        else:
            return True
