# -*- coding: utf-8 -*-
"""
Created on Sun Aug 29 14:05:57 2021

@author: Balazs
"""

class component_class():
    def __init__(self):
        self.__add_value = 1
        self.store_value = 2
    def do_method(self):
        self.store_value = self.store_value + self.__add_value


cc1 = component_class()

cc1.store_value

cc1.do_method()


class main_class():
    def __init__(self):
        self.value1 = 1        
        self.value2 = 2
        self.component = component_class()
        
        
        
myclass = main_class()


myclass.component.store_value

myclass.component.__add_value

myclass.component.store_value

myclass.component.do_method()
