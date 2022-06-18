#!/usr/bin/env python3

import time
import threading


def boil_udon():
  print('  うどんを茹でます。')
  time.sleep(3)
  print('  うどんが茹であがりました。')


def make_tuyu():
  print('  ツユをつくります。')
  time.sleep(2)
  print('  ツユができました。')

'''
#逐次処理

print('うどんを作ります。')
boil_udon()
make_tuyu()
print('盛り付けます。')
print('うどんができました。')
'''

#並列処理

print('うどんを作ります。')
thread1 = threading.Thread(target=boil_udon)
thread2 = threading.Thread(target=make_tuyu)
thread1.start() #スレッド開始
thread2.start()
thread1.join() #スレッド終了まで待つ
thread2.join()
print('盛り付けます。')
print('うどんができました。')
