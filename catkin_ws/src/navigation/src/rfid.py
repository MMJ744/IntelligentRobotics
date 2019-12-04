#!/usr/bin/env python
import nfc
from nfc.clf import RemoteTarget

#print(clf.open('usb:001:059'))
#print(clf.open('usb:072f:2200'))
#print(clf.open('usb:001'))
#print(clf.open('usb:072f'))
#print(clf.open('usb'))

#Blank = 66411D25
#Red = 66C8AA25
#Blue = 86C41A25
#Yellow = 76BBEB25
#Black = 76DA3725
def readCard():
    try:
        customers = {'66411D25': 'Blank Boi', '66C8AA25': 'Brexit Means Brexit', '86C41A25': 'The Flying Scotsman', '76BBEB25': 'Barry Bee Benson', '76DA3725': 'Whatever you want'}
        clf = nfc.ContactlessFrontend('usb')
        tag = clf.connect(rdwr={'on-connect': lambda tag: False})
        #print(tag)
        id = str(tag)[12::]
        clf.close()
        if id in customers:
            return customers[id]
    except:
        print("SOMETHING WENT WRONG WITH CARD READING")
    return None