import numpy as np
import scipy as sc

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
    def __init__(self, y: function, dy: function, z: function, dz: function):
        self.y = y
        self.dy = dy
        self.z = z
        self.dz = dz
        
        self.cached_length = None
        self.cached_coa = None
        self.cached_second_moment = None
    
    #derivative of z wrt y at a point along the curve
    def dz_dy(self, t: float):
        return self.dz(t) / self.dy(t)
    
    #change in line length as t vaies
    def dL(self, t:float) -> float:
        return np.sqrt(self.dy(t)**2 + self.dz(t)**2)
    
    @property
    def length(self) -> float:
        if self.cached_length == None:
            self.cached_length = sc.integrate.quad(self.dL, 0, 1)[0]
        return self.cached_length
    
    # center of area (average y and z coordinates)
    @property
    def coa(self) -> np.array:
        if self.cached_coa == None:
            self.cached_coa = np.zeros(2)
            self.cached_coa[0] = sc.integrate.quad(lambda t: self.y(t) * self.dL(t), 0, 1)[0]
            self.cached_coa[1] = sc.integrate.quad(lambda t: self.z(t) * self.dL(t), 0, 1)[0]
            self.cached_coa /= self.length
        return self.cached_coa
    
    @property
    def second_moment(self) -> np.array:
        if self.cached_second_moment == None:
            y_sqr_int = sc.integrate.quad(lambda t: (self.y(t) - self.coa[0])**2 * self.dl(t), 0, 1)[0]
            z_sqr_int = sc.integrate.quad(lambda t: (self.z(t) - self.coa[1])**2 * self.dL(t), 0, 1)[0]
            yz_prod = -sc.integrate.quad(lambda t: (self.y(t) - self.coa[0]) * (self.z(t) - self.coa[1]) * self.dL(t), 0, 1)
            self.cached_second_moment = np.array([y_sqr_int + z_sqr_int, z_sqr_int, y_sqr_int, 0, 0, yz_prod])
        return self.cached_second_moment

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
        y = lambda t: del_y * t + y0
        dy = lambda t: del_y
        z = lambda t: del_z * t + z0
        dz = lambda t: del_z
        super().__init__(y, dy, z, dz)

class Cubic_Bezier(Curve):
    def __init__(self, p0: np.array, p1: np.array, p2: np.array, p3: np.array):
        self.p0 = p0
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        def y(t):
            return (1 - t)**3 * p0[0] + 3*(1 - t)**2 * t * p1[0] + 3*(1 - t) * t**2 * p2[0] + t**3  * p3[0]
        
        def dy(t):
            return 3 * (1 - t)**2 * (p1[0] - p0[0]) + 6 * (1 - t) * t * (p2[0] - p1[0]) + 3 * t**2 * (p3[0] - p2[0])
        
        def z(t):
            return (1 - t)**3 * p0[1] + 3*(1 - t)**2 * t * p1[1] + 3*(1 - t) * t**2 * p2[1] + t**3  * p3[1]
        
        def dz(t):
            return 3 * (1 - t)**2 * (p1[1] - p0[1]) + 6 * (1 - t) * t * (p2[1] - p1[1]) + 3 * t**2 * (p3[1] - p2[1])
        
        super().__init__(y, dy, z, dz)

class Bounded_Region(Geometry):
    '''
    A region bounded by two curves in the yz plane
    The y values of the curve must be equal at t = 0 and t = 1
    '''
    def __init__(self, upper_boundary: Curve, lower_boundary: Curve):
        if upper_boundary.y(0) == lower_boundary.y(0) and upper_boundary.y(1) == lower_boundary.y(1):
            self.upper = upper_boundary
            self.lower = lower_boundary
        else:
            raise ValueError("Curves must have the same start and ending y values")
        
        self.cached_area = None
        
    def dividing_line(self, t: float, upper = True) -> float:
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
    
    @property
    def area(self):
        if self.cached_area == None:
            self.cached_area = sc.integrate.quad(lambda t: (self.upper.z(t) - self.dividing_line(t)) * self.upper.dy(t), 0, 1)[0]
            self.cached_area += sc.integrate.quad(lambda t: (self.dividing_line(t, upper=False) - self.lower.z(t)) * self.lower.dy(t), 0, 1)[0]
        return self.cached_area
    
    # center of area (average y and z coordinates)
    @property
    def coa(self) -> np.array:
        if self.cached_coa == None:
            coa_y_upper = sc.integrate.quad(lambda t: self.upper.y(t) * (self.upper.z(t) - self.dividing_line(t)) * self.upper.dy(t), 0, 1)[0]
            coa_y_lower = sc.integrate.quad(lambda t: self.lower.y(t) * (self.dividing_line(t, upper=False) - self.lower.z(t)) * self.lower.dy(t), 0, 1)[0]
            
            coa_z_upper = 0.5 * sc.integrate.quad(lambda t: (self.upper.z(t) - self.dividing_line(t))**2 * self.upper.dy(t), 0, 1)[0]
            coa_z_lower = 0.5 * sc.integrate.quad(lambda t: (self.dividing_line(t, upper=False) - self.lower.z(t))**2 * self.lower.dy(t), 0, 1)[0]
            
            self.cached_coa = np.array([coa_y_lower + coa_y_upper, coa_z_lower + coa_z_upper]) / self.area
        return self.cached_coa
    
    @property
    def second_moment(self) -> np.array:
        #suffering
        if self.cached_second_moment == None:
            y_sqr_int_upper = sc.integrate.quad(lambda t: (self.upper.y(t) - self.coa[0])**2 * (self.upper.z(t) - self.dividing_line(t)) * self.upper.dy(t), 0, 1)[0]
            y_sqr_int_lower = sc.integrate.quad(lambda t: (self.lower.y(t) - self.coa[0])**2 * (self.dividing_line(t, upper=False) - self.lower.z(t)) * self.lower.dy(t), 0, 1)[0]
            y_sqr_int = y_sqr_int_lower + y_sqr_int_upper
            
            z_sqr_int_upper = 1/3 * sc.integrate.quad(lambda t: ((self.upper.z(t) - self.coa[1])**3 - (self.dividing_line(t) - self.coa[1])**3) * self.upper.dy(t), 0, 1)[0]
            z_sqr_int_lower = 1/3 * sc.integrate.quad(lambda t: ((self.dividing_line(t, upper=False) - self.coa[1])**3 - (self.lower.z(t) - self.coa[1])**3) * self.lower.dy(t), 0, 1)[0]
            z_sqr_int = z_sqr_int_lower + z_sqr_int_upper
            
            yz_prod_upper = -0.5 * sc.integrate.quad(lambda t: ((self.upper.z(t) - self.coa[1])**2 - (self.dividing_line(t) - self.coa[1])**2) * self.upper.y(t) * self.upper.dy(t), 0, 1)[0]
            yz_prod_lower = -0.5 * sc.integrate.quad(lambda t: ((self.dividing_line(t, upper=False) - self.coa[1])**2 - (self.lower.z(t) - self.coa[1])**2) * self.lower.y(t) * self.lower.dy(t), 0, 1)[0]
            yz_prod = yz_prod_upper + yz_prod_lower
            
            self.cached_second_moment = np.array([y_sqr_int + z_sqr_int, z_sqr_int, y_sqr_int, 0, 0, yz_prod])
        return self.cached_second_moment

if __name__ == "__main__":
    print("hi")