import operator

import numpy as np
from Quaternions_old import Quaternions

class Animation:
    """
    Animation is a numpy-like wrapper for animation data
    
    Animation data consists of several arrays consisting
    of F frames and J joints.
    
    The animation is specified by
    
        rotations : (F, J) Quaternions | Joint Rotations
        positions : (F, J, 3) ndarray  | Joint Positions
    
    The base pose is specified by
    
        orients   : (J) Quaternions    | Joint Orientations
        offsets   : (J, 3) ndarray     | Joint Offsets
        
    And the skeletal structure is specified by
        
        parents   : (J) ndarray        | Joint Parents
    """
    
    def __init__(self, rotations, positions, orients, offsets, parents):
        
        self.rotations = rotations
        self.positions = positions
        self.orients   = orients
        self.offsets   = offsets
        self.parents   = parents
    
    def __op__(self, op, other):
        return Animation(
            op(self.rotations, other.rotations),
            op(self.positions, other.positions),
            op(self.orients, other.orients),
            op(self.offsets, other.offsets),
            op(self.parents, other.parents))

    def __iop__(self, op, other):
        self.rotations = op(self.roations, other.rotations)
        self.positions = op(self.roations, other.positions)
        self.orients   = op(self.orients, other.orients)
        self.offsets   = op(self.offsets, other.offsets)
        self.parents   = op(self.parents, other.parents)
        return self
    
    def __sop__(self, op):
        return Animation(
            op(self.rotations),
            op(self.positions),
            op(self.orients),
            op(self.offsets),
            op(self.parents))
    
    def __add__(self, other): return self.__op__(operator.add, other)
    def __sub__(self, other): return self.__op__(operator.sub, other)
    def __mul__(self, other): return self.__op__(operator.mul, other)
    def __div__(self, other): return self.__op__(operator.div, other)
    
    def __abs__(self): return self.__sop__(operator.abs)
    def __neg__(self): return self.__sop__(operator.neg)
    
    def __iadd__(self, other): return self.__iop__(operator.iadd, other)
    def __isub__(self, other): return self.__iop__(operator.isub, other)
    def __imul__(self, other): return self.__iop__(operator.imul, other)
    def __idiv__(self, other): return self.__iop__(operator.idiv, other)
    
    def __len__(self): return len(self.rotations)
    
    def __getitem__(self, k):
        if isinstance(k, tuple):
            return Animation(
                self.rotations[k],
                self.positions[k],
                self.orients[k[1:]],
                self.offsets[k[1:]],
                self.parents[k[1:]]) 
        else:
            return Animation(
                self.rotations[k],
                self.positions[k],
                self.orients,
                self.offsets,
                self.parents) 
        
    def __setitem__(self, k, v): 
        if isinstance(k, tuple):
            self.rotations.__setitem__(k, v.rotations)
            self.positions.__setitem__(k, v.positions)
            self.orients.__setitem__(k[1:], v.orients)
            self.offsets.__setitem__(k[1:], v.offsets)
            self.parents.__setitem__(k[1:], v.parents)
        else:
            self.rotations.__setitem__(k, v.rotations)
            self.positions.__setitem__(k, v.positions)
            self.orients.__setitem__(k, v.orients)
            self.offsets.__setitem__(k, v.offsets)
            self.parents.__setitem__(k, v.parents)
        
    @property
    def shape(self): return (self.rotations.shape[0], self.rotations.shape[1])
            
    def copy(self): return Animation(
        self.rotations.copy(), self.positions.copy(), 
        self.orients.copy(), self.offsets.copy(), 
        self.parents.copy())
    
    def repeat(self, *args, **kw):
        return Animation(
            self.rotations.repeat(*args, **kw),
            self.positions.repeat(*args, **kw),
            self.orients, self.offsets, self.parents)   
    


