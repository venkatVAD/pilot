# import threading
 
# class MyClass:
#     def __init__(self):
        
#         somevar = 'someval'
#         self.i=0
#         self.j=0


#     def f1(self):
#         while True:
#             self.i += 1
#     def f2(self):
#         while True:
#             self.j = 5
   

#     def func_to_be_threaded(self):
#         threading.Thread(target=self.f1).start()
#         threading.Thread(target=self.f2).start()

# if __name__=='__main__':
#      m=MyClass()
#      m.func_to_be_threaded()
#      while True:
#          print('values I = '+str(m.i)+" J ="+str(m.j))


# from pilot import trottle_send, stering_send

# trottle_send(500) # RPM 
# stering_send(180)
# trolltle_start_cheak()
# from time import time

# x = time()
# print(x)
drive_dirctn={'status' : None ,"data" : {'forward' : None, 'reverse': None, 'seat_s' : None},'time' : None}
drive_dirctn['data']['forward']=5
print(drive_dirctn['data']['forward'])
