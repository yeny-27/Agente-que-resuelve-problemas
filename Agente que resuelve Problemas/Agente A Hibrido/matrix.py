from math import *
import random

class matrix:
    
    def __init__(self, value):
        self.value = value
        self.dimx  = len(value)
        self.dimy  = len(value[0])
        if value == [[]]:
            self.dimx = 0

    def zero(self, dimx, dimy):
        # verificar dimensiones
        if dimx < 1 or dimy < 1:
            raise ValueError, "Tamaño de matriz no válido"
        else:
            self.dimx  = dimx
            self.dimy  = dimy
            self.value = [[0 for row in range(dimy)] for col in range(dimx)]

    def identity(self, dim):
        # Verificar dimensiones
        if dim < 1:
            raise ValueError, "Tamaño de matriz no válido"
        else:
            self.dimx  = dim
            self.dimy  = dim
            self.value = [[0 for row in range(dim)] for col in range(dim)]
            for i in range(dim):
                self.value[i][i] = 1

    def show(self):
        for i in range(self.dimx):
            print self.value[i]
        print ' '


    def __sum__(self, other):
        if self.dimx != other.dimx or self.dimx != other.dimx:
            raise ValueError, "Las matrices deben ser de igual dimensión para sumar"
        else:
            # suma si las dimeniones son correctas
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] + other.value[i][j]
            return res

    def __res__(self, other):
        # verifica el tamaño correcto
        if self.dimx != other.dimx or self.dimx != other.dimx:
            raise ValueError, "Las matrices deben ser de igual dimensión para restarse"
        else:
            # res si la dimension es correcta
            res = matrix([[]])
            res.zero(self.dimx, self.dimy)
            for i in range(self.dimx):
                for j in range(self.dimy):
                    res.value[i][j] = self.value[i][j] - other.value[i][j]
            return res

    def __mul__(self, other):
        # verificar dimensiones
        if self.dimy != other.dimx:
            raise ValueError, "m*n o n*p para multiplicar"
        else:
            # multiplicar si las dimensiones son correctas
            res = matrix([[]])
            res.zero(self.dimx, other.dimy)
            for i in range(self.dimx):
                for j in range(other.dimy):
                    for k in range(self.dimy):
                        res.value[i][j] += self.value[i][k] * other.value[k][j]
        return res

    def transpose(self):
        # traspuesta de toda la vida xd
        res = matrix([[]])
        res.zero(self.dimy, self.dimx)
        for i in range(self.dimx):
            for j in range(self.dimy):
                res.value[j][i] = self.value[i][j]
        return res


    def __repr__(self):
        return repr(self.value)
