# -*- coding: utf-8 -*-
"""
Created on Fri Jul  9 10:01:34 2021

@author: Balazs
"""

import krpc

krpc.__version__

#connection = krpc.connect(
#    address= '127.0.0.1',
#    rpc_port = 5000,
#    stream_port=50001)

connection = krpc.connect()
vessel = connection.space_center.active_vessel

vessel.control.activate_next_stage()


import krpc
conn = krpc.connect(name='Hello World')
vessel = conn.space_center.active_vessel
print(vessel.name)
