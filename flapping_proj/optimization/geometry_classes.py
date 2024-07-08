import numpy as np
import scipy as sc
from utils.message import Message


def almost_equal(a, b, error_percent):
    return abs(a - b) <= abs(a*error_percent)

# I feel like this is needed but don't know what to do with it


class Geometry():
    pass
    # this may be useful in the distant future to make this a generic
    # 3d object
    # Without fully understanding how classes work I don't want to
    # add it in.
    # def __init__(self, volume, cov, second_moment) -> None:
    #     self.volume = volume
    #     self.cov = cov
    #     self.second_moment = second_moment


class Curve(Geometry):
    '''
    object containing a parametric function of t in the yz plane
    for the curve, t should be between 0 and 1
    y: y coordiante as a function of t
    z: z coordinate as a function of t
    dy: derivative of y with respect to t
    dz: derivative of z with respect to t

    all functions should be of the form func(t) and return a float
    z must be unique for any given value of y
    '''

    def __init__(self, y: callable, dy: callable, z: callable, dz: callable):
        self.y = y
        self.dy = dy
        self.z = z
        self.dz = dz

        self.length = sc.integrate.quad(self.dL, 0, 1)[0]

        # coa = center of area
        self.coa = np.zeros(2)
        self.coa[0] = sc.integrate.quad(
            lambda t: self.y(t) * self.dL(t), 0, 1)[0]
        self.coa[1] = sc.integrate.quad(
            lambda t: self.z(t) * self.dL(t), 0, 1)[0]
        self.coa /= self.length

        y_sqr_int = sc.integrate.quad(lambda t: (
            self.y(t) - self.coa[0])**2 * self.dL(t), 0, 1)[0]
        z_sqr_int = sc.integrate.quad(lambda t: (
            self.z(t) - self.coa[1])**2 * self.dL(t), 0, 1)[0]
        yz_prod = -sc.integrate.quad(lambda t: (self.y(t) - self.coa[0]) * (
            self.z(t) - self.coa[1]) * self.dL(t), 0, 1)[0]
        self.second_moment = np.array(
            [y_sqr_int + z_sqr_int, z_sqr_int, y_sqr_int, 0, 0, yz_prod])

    # derivative of z wrt y at a point along the curve

    def dz_dy(self, t: float):
        return self.dz(t) / self.dy(t)

    # change in line length as t vaies
    def dL(self, t: float) -> float:
        return np.sqrt(self.dy(t)**2 + self.dz(t)**2)

# special curve case for a straight line


class Line(Curve):
    '''
    definition of a line in the yz plane
    (y0, z0) defines the first point
    (y1, z1) defines the second point
    '''

    def __init__(self, y0: float, z0: float, y1: float, z1: float):
        self.p0 = np.array([y0, z0])
        self.p1 = np.array([y1, z1])

        del_y = y1 - y0
        del_z = z1 - z0
        def y(t): return del_y * t + y0
        def dy(t): return del_y
        def z(t): return del_z * t + z0
        def dz(t): return del_z
        super().__init__(y, dy, z, dz)


class Cubic_Bezier(Curve):
    def __init__(self, p0: np.ndarray, p1: np.ndarray, p2: np.ndarray, p3: np.ndarray):
        self.p0 = p0
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3

        def y(t):
            return (1 - t)**3 * p0[0] + 3*(1 - t)**2 * t * p1[0] + 3*(1 - t) * t**2 * p2[0] + t**3 * p3[0]

        def dy(t):
            return 3 * (1 - t)**2 * (p1[0] - p0[0]) + 6 * (1 - t) * t * (p2[0] - p1[0]) + 3 * t**2 * (p3[0] - p2[0])

        def z(t):
            return (1 - t)**3 * p0[1] + 3*(1 - t)**2 * t * p1[1] + 3*(1 - t) * t**2 * p2[1] + t**3 * p3[1]

        def dz(t):
            return 3 * (1 - t)**2 * (p1[1] - p0[1]) + 6 * (1 - t) * t * (p2[1] - p1[1]) + 3 * t**2 * (p3[1] - p2[1])

        super().__init__(y, dy, z, dz)


class LineSegments(Curve):
    def __init__(self, y_points: np.ndarray, z_points: np.ndarray) -> None:
        self.y_points = np.asarray(y_points)
        self.z_points = np.asarray(z_points)
        num_segments = len(y_points) - 1
        if num_segments < 2:
            raise Exception(
                "Line_Segments instances has less than 2 segments. Please use Line instead")
        if num_segments + 1 != len(z_points):
            raise ValueError(
                "Number of y coordinates does not match the number of z coordinates")
        for i in range(num_segments - 1):
            if y_points[i] > y_points[i + 1]:
                raise ValueError("y coordinates may not decrease")

        def y(t: float):
            index = (np.floor(t * num_segments) - np.floor(t)).astype(int)
            del_y = y_points[index + 1] - y_points[index]
            return del_y * (num_segments * t - index) + y_points[index]

        def dy(t: float):
            index = (np.floor(t * num_segments) - np.floor(t)).astype(int)
            del_y = y_points[index + 1] - y_points[index]
            return num_segments * del_y

        def z(t: float):
            index = int((np.floor(t * num_segments) - np.floor(t)))
            del_z = z_points[index + 1] - z_points[index]
            return del_z * (num_segments * t - index) + z_points[index]

        def dz(t: float):
            index = (np.floor(t * num_segments) - np.floor(t)).astype(int)
            del_z = z_points[index + 1] - z_points[index]
            return num_segments * del_z

        super().__init__(y, dy, z, dz)


class BoundedRegion(Geometry):
    '''
    A region bounded by two curves in the yz plane
    The y values of the curve must be equal at t = 0 and t = 1      
    '''

    def __init__(self, upper_boundary: Curve, lower_boundary: Curve):
        if almost_equal(upper_boundary.y(0), lower_boundary.y(0), 1e-12) and almost_equal(upper_boundary.y(1), lower_boundary.y(1), 1e-12):
            self.upper = upper_boundary
            self.lower = lower_boundary
        else:
            raise ValueError(
                f"Curves must have the same start and ending y values. Start ys: {upper_boundary.y(0)}, {lower_boundary.y(0)}. End ys: {upper_boundary.y(1)}, {lower_boundary.y(1)}")

        # area
        self.area = sc.integrate.quad(lambda t: (self.upper.z(
            t) - self.dividing_line(t)) * self.upper.dy(t), 0, 1)[0]
        self.area += sc.integrate.quad(lambda t: (self.dividing_line(
            t, upper=False) - self.lower.z(t)) * self.lower.dy(t), 0, 1)[0]

        # center of area
        coa_y_upper = sc.integrate.quad(lambda t: self.upper.y(
            t) * (self.upper.z(t) - self.dividing_line(t)) * self.upper.dy(t), 0, 1)[0]
        coa_y_lower = sc.integrate.quad(lambda t: self.lower.y(
            t) * (self.dividing_line(t, upper=False) - self.lower.z(t)) * self.lower.dy(t), 0, 1)[0]

        coa_z_upper = 0.5 * sc.integrate.quad(lambda t: (self.upper.z(
            t) - self.dividing_line(t))**2 * self.upper.dy(t), 0, 1)[0]
        coa_z_lower = 0.5 * sc.integrate.quad(lambda t: (self.dividing_line(
            t, upper=False) - self.lower.z(t))**2 * self.lower.dy(t), 0, 1)[0]

        self.coa = np.array(
            [coa_y_lower + coa_y_upper, coa_z_lower + coa_z_upper]) / self.area

        # second area moment
        y_sqr_int_upper = sc.integrate.quad(lambda t: (self.upper.y(
            t) - self.coa[0])**2 * (self.upper.z(t) - self.dividing_line(t)) * self.upper.dy(t), 0, 1)[0]
        y_sqr_int_lower = sc.integrate.quad(lambda t: (self.lower.y(t) - self.coa[0])**2 * (
            self.dividing_line(t, upper=False) - self.lower.z(t)) * self.lower.dy(t), 0, 1)[0]
        y_sqr_int = y_sqr_int_lower + y_sqr_int_upper

        z_sqr_int_upper = 1/3 * sc.integrate.quad(lambda t: ((self.upper.z(t) - self.coa[1])**3 - (
            self.dividing_line(t) - self.coa[1])**3) * self.upper.dy(t), 0, 1)[0]
        z_sqr_int_lower = 1/3 * sc.integrate.quad(lambda t: ((self.dividing_line(
            t, upper=False) - self.coa[1])**3 - (self.lower.z(t) - self.coa[1])**3) * self.lower.dy(t), 0, 1)[0]
        z_sqr_int = z_sqr_int_lower + z_sqr_int_upper

        yz_prod_upper = -0.5 * sc.integrate.quad(lambda t: ((self.upper.z(t) - self.coa[1])**2 - (
            self.dividing_line(t) - self.coa[1])**2) * self.upper.y(t) * self.upper.dy(t), 0, 1)[0]
        yz_prod_lower = -0.5 * sc.integrate.quad(lambda t: ((self.dividing_line(t, upper=False) - self.coa[1])**2 - (
            self.lower.z(t) - self.coa[1])**2) * self.lower.y(t) * self.lower.dy(t), 0, 1)[0]
        yz_prod = yz_prod_upper + yz_prod_lower

        self.second_moment = np.array(
            [y_sqr_int + z_sqr_int, z_sqr_int, y_sqr_int, 0, 0, yz_prod])

    def dividing_line(self, t: float, upper=True) -> float:
        '''
        function that returns the z coordinate of a line that splits the 
        region into upper and lower sections for a given t. The necessary
        z coordinate changes depending on if the dividing line is being investigated
        relative to the upper boundary or lower boundary as t does not have to map
        to the same y value for both boundaries. 

        upper = True means the reference is the upper curve. Otherwise it is the lower curve.
        '''
        relative_curve = self.lower
        if upper:
            relative_curve = self.upper

        del_y = self.upper.y(1) - self.upper.y(0)
        del_z = self.upper.z(1) - self.lower.z(0)

        return del_z / del_y * relative_curve.y(t) - self.lower.z(0)


class Polygon(BoundedRegion):
    def __init__(self, n: int, diameter: float):
        if n < 3:
            raise ValueError("Polygon must have at least three sides")
        ys = np.zeros(int(np.ceil(n / 2) + 1))
        top_zs = np.zeros_like(ys)
        angle = 2 * np.pi / n
        point_radius = diameter / np.cos(angle / 2) / 2
        for i in range(int(n / 2) + 1):
            ys[i] = point_radius * np.cos(np.pi - i * angle)
            top_zs[i] = point_radius * np.sin(np.pi - i * angle)
        if n % 2 == 1:
            ys[-1] = diameter / 2
            top_zs[-1] = 0

        super().__init__(LineSegments(ys, top_zs), LineSegments(ys, -top_zs))


class Circle(BoundedRegion):
    def __init__(self, diameter: float):
        def y(t):
            return diameter / 2 * np.cos(np.pi - np.pi * t)

        def dy(t):
            return diameter * np.pi / 2 * np.sin(np.pi - np.pi * t)

        def z(t):
            return diameter / 2 * np.sin(np.pi - np.pi * t)

        def dz(t):
            return -diameter * np.pi / 2 * np.cos(np.pi - np.pi * t)

        super().__init__(Curve(y, dy, z, dz), Curve(
            y, dy, lambda t: -z(t), lambda t: -dz(t)))


class Rectangle(BoundedRegion):
    def __init__(self, base: float, height: float):
        half_base = base / 2
        half_height = height / 2
        upper_boundary = LineSegments(
            [-half_base, - half_base, half_base, half_base], [0, half_height, half_height, 0])
        lower_boundary = LineSegments(
            [-half_base, - half_base, half_base, half_base], [0, -half_height, -half_height, 0])
        super().__init__(upper_boundary, lower_boundary)


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    t = np.linspace(0, 1, 1000)
    circle = Rectangle(6, 1)
    print(circle.area)
    ys = np.zeros_like(t)
    zs = np.zeros_like(t)
    dys = np.zeros_like(t)
    dzs = np.zeros_like(t)
    for i in range(len(t)):
        ys[i] = circle.upper.y(t[i])
        zs[i] = circle.upper.z(t[i])
        dys[i] = circle.lower.y(t[i])
        dzs[i] = circle.lower.z(t[i])
    plt.scatter(ys, zs, c=t, cmap="cool")
    plt.scatter(dys, dzs, c=t, cmap="hot")
