from math import hypot, sqrt, asin
from dynamic_graph.sot.tools.se3 import SO3
 
class Quaternion (object):
    def __init__(self,a,b):
        self.a=a
        self.b=b
 
    def __str__(self):
        aff='('
        aff+=str(self.a.real)+')+('
        aff+=str(self.a.imag)+')i+('
        aff+=str(self.b.real)+')j+('
        aff+=str(self.b.imag)+')k'
        return aff
 
    def __neg__(self):
        return Quaternion(-self.a,-self.b)
 
    def __add__(self,other):
        return Quaternion(self.a+other.a,self.b+other.b)
 
    def __sub__(self,other):
        return Quaternion(self.a-other.a,self.b-other.b)
 
    def __mul__(self,other):
        c=self.a*other.a-self.b*other.b.conjugate()
        d=self.a*other.b+self.b*other.a.conjugate()
        return Quaternion(c,d)
 
    def __rmul__(self,k):
        return Quaternion(self.a*k,self.b*k)
 
    def __abs__(self):
        return hypot(abs(self.a),abs(self.b))
 
    def conjugate(self):
        return Quaternion(self.a.conjugate(),-self.b)
 
    def __div__(self,other):
        return self*(1./abs(other)**2*other.conjugate ())
 
    def __pow__(self,n):
        r=1
        for i in range(n):
                r=r*self
        return r

    def normalize (self):
        norm = abs (self)
        self.a /= norm;
        self.b /= norm;
        return self

    def utheta (self):
        a = self.a.real
        b = self.a.imag
        c = self.b.real
        d = self.b.imag
        norm = sqrt (b*b + c*c + d*d)
        theta = 2 * asin (norm)
        if theta > 1e-6:
            return [theta*b/norm, theta*c/norm, theta*d/norm]
        else:
            return [2*b, 2*c, 2*d]

    def toMatrix (self):
        a = self.a.real
        b = self.a.imag
        c = self.b.real
        d = self.b.imag
        R = []
        row = []
        row.append (a*a + b*b - c*c - d*d)
        row.append (2*b*c + 2*a*d)
        row.append (2*b*d - 2*a*c)
        R.append (row); row = []
        row.append (2*b*c - 2*a*d)
        row.append (a*a - b*b + c*c - d*d)
        row.append (2*c*d + 2*a*b)
        R.append (row); row = []
        row.append (2*b*d + 2*a*c)
        row.append (2*c*d - 2*a*b)
        row.append (a*a - b*b - c*c + d*d)
        R.append (row);
        return tuple (map (tuple, R))

    def toSO3 (self):
        return SO3 (self.toMatrix ())
