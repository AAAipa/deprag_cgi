#!/usr/bin/env python

import rospy
from deprag_cgi.srv import *
import requests as requester




ip = "10.10.10.10" #ip address of screwdriver
max_torque = 150 #maximum torque of screwdriver
min_rpm = 0         #minimum driver rpm
max_rpm = 1500      #max driver rpm
max_torque_hold = 30000 #max torque hold time

InstanceList = [] #hold all ongoing programm instances

class Program: #holds information of a cgi instance
    def __init__(self,id):
        self.ID = id
        self.stepnumber = 1
        self.reqString = "http://" + ip + "/cgi-bin/cgiread?site=p"
    def SN(self): #convert stepnumber into a cgi friendly format
        ones = self.stepnumber % 10
        tenths = (self.stepnumber - ones) / 10
        return ( str(tenths) + str(ones) )

def GenerateID(): #finds the smalest id that is not yet taken
    uid = -1
    unique = False
    while(unique == False):
        uid = uid + 1
        localCheck = True
        for obj in InstanceList:
            if(obj.ID == uid):
                localCheck = False
        unique = localCheck
    return uid

def IndexForID(id): #Finds the coresponding list index to a given ID
    for idx, obj in enumerate(InstanceList):
        if(obj.ID == id):
            return idx
    return -1  #no object matches id

def Create(req):  #create a new CGI-instance
    res = create_pgmResponse()
    res.id = GenerateID()  #negative ID indicates an error

    handle = Program(res.id)
    
    handle.reqString = handle.reqString + "&program=" + str(req.pgm_number) #insert program number
    handle.reqString = handle.reqString + "&head_title=" + req.head_title #insert title of 
    
    if(req.head_dir > 1): #check if head dir is valid value
        res.id = -1
        return res

    handle.reqString = handle.reqString + "&head_dir=" + str(req.head_dir)

    if(req.head_feed > 2): #check if head_feed is a valid value !!! when no feed use 2
        res.id = -2
        return res

    handle.reqString = handle.reqString + "&head_feed=" + str(req.head_feed)

    if(req.torque_unit > 5): #check if torque unit is valid value
        res.id = -3
        return res

    handle.reqString = handle.reqString + "&torqueunit=" + str(req.torque_unit)

    InstanceList.append(handle) #add to global list of programms

    return res

def TightTorqueUnsup(req):
    #setup
    ind = IndexForID(req.cid)
    res = tighten_torque_unsupervisedResponse()
    res.id = req.cid
    if(ind < 0):
        res.id = -4 #couldnt find id
        return res
    handle = InstanceList[ind]
    predex = "&p"+handle.SN()+"_"

    #set Programm Type
    handle.reqString = handle.reqString + predex + "block=1"
    handle.reqString = handle.reqString + predex + "num=1"

    #set max screwdriving time
    if(req.max_time > 65535):
        res.id = -5 #max time to high
        return res
    handle.reqString = handle.reqString + predex + "1=" + str(req.max_time)

    #set shutoff torque
    if(req.torque_cutoff > max_torque or req.torque_cutoff < 0):
        res.id = -6 #torque cutoff to high or negative
    handle.reqString = handle.reqString + predex + "2=" + "{:.1f}".format(req.torque_cutoff)  #TODO check if it must be rounded

    #set min torque
    if(req.torque_min > max_torque or req.torque_min < 0):
        res.id = -7 #torque min to high or negative
    handle.reqString = handle.reqString + predex + "3=" + "{:.1f}".format(req.torque_min) #TODO check if it must be rounded

    #set max torque
    if(req.torque_max > max_torque or req.torque_max < 0):
        res.id = -8 #torque cutoff to high or negative
    handle.reqString = handle.reqString + predex + "4=" + "{:.1f}".format(req.torque_max) #TODO check if it must be rounded

    #set screwdriver speed
    if(req.rpm > max_rpm or req.rpm < min_rpm):
        res.id = -9 #scredriver speed not legal
        return res
    handle.reqString = handle.reqString + predex + "10=" + str(req.rpm)   

    #set torque hold time 
    if(req.torque_hold > max_torque_hold):
        res.id = -10 #torque hold to high
        return res
    handle.reqString = handle.reqString + predex + "9=" + str(req.torque_hold) 

    handle.stepnumber = handle.stepnumber + 1
    return res 

def Publish(req):
    #setup
    ind = IndexForID(req.cid)
    res = publishResponse()
    res.id = req.cid
    if(ind < 0):
        res.id = -11 #couldnt find id
        return res
    handle = InstanceList[ind]

    _str = handle.reqString #'http://www.tutorialspoint.com/python/' for debuging
    r = requester.get(_str)
    res.response = r.text
    print(_str)  #for debuging can be disabled
    del InstanceList[ind]
    return res

def DispValues(req):
    #setup
    ind = IndexForID(req.cid)
    res = secondary_attributeResponse()
    res.id = req.cid
    if(ind < 0):
        res.id = -12 #couldnt find id
        return res
    handle = InstanceList[ind]
    predex = "&p"+handle.SN()+"_"

    #set Programm Type
    handle.reqString = handle.reqString + predex + "block=24"
    handle.reqString = handle.reqString + predex + "num=1"

    handle.stepnumber = handle.stepnumber + 1
    return res

def SaveValues(req):
    #setup
    ind = IndexForID(req.cid)
    res = secondary_attributeResponse()
    res.id = req.cid
    if(ind < 0):
        res.id = -13 #couldnt find id
        return res
    handle = InstanceList[ind]
    predex = "&p"+handle.SN()+"_"

    #set Programm Type
    handle.reqString = handle.reqString + predex + "block=25"
    handle.reqString = handle.reqString + predex + "num=1"

    handle.stepnumber = handle.stepnumber + 1
    return res
    
def Statistics(req):
    #setup
    ind = IndexForID(req.cid)
    res = secondary_attributeResponse()
    res.id = req.cid
    if(ind < 0):
        res.id = -14 #couldnt find id
        return res
    handle = InstanceList[ind]
    predex = "&p"+handle.SN()+"_"

    #set Programm Type
    handle.reqString = handle.reqString + predex + "block=26"
    handle.reqString = handle.reqString + predex + "num=1"

    handle.stepnumber = handle.stepnumber + 1
    return res

def LoosenTorqueUnsup(req):
    #setup
    ind = IndexForID(req.cid)
    res = loosen_torque_unsupervisedResponse()
    res.id = req.cid
    if(ind < 0):
        res.id = -15 #couldnt find id
        return res
    handle = InstanceList[ind]
    predex = "&p"+handle.SN()+"_"

    #set Programm Type
    handle.reqString = handle.reqString + predex + "block=6"
    handle.reqString = handle.reqString + predex + "num=1"

    #set max screwdriving time
    if(req.max_time > 65535):
        res.id = -16 #max time to high
        return res
    handle.reqString = handle.reqString + predex + "1=" + str(req.max_time)

    #set shutoff torque
    if(req.torque_cutoff > max_torque or req.torque_cutoff < 0):
        res.id = -17 #torque cutoff to high or negative
    handle.reqString = handle.reqString + predex + "2=" + "{:.1f}".format(req.torque_cutoff)  #TODO check if it must be rounded

    #set min torque
    if(req.torque_min > max_torque or req.torque_min < 0):
        res.id = -18 #torque min to high or negative
    handle.reqString = handle.reqString + predex + "3=" + "{:.1f}".format(req.torque_min) #TODO check if it must be rounded

    #set max torque
    if(req.torque_max > max_torque or req.torque_max < 0):
        res.id = -19 #torque cutoff to high or negative
    handle.reqString = handle.reqString + predex + "4=" + "{:.1f}".format(req.torque_max) #TODO check if it must be rounded

    #set screwdriver speed
    if(req.rpm > max_rpm or req.rpm < min_rpm):
        res.id = -20 #scredriver speed not legal
        return res
    handle.reqString = handle.reqString + predex + "10=" + str(req.rpm)   

    #set torque hold time 
    if(req.torque_hold > max_torque_hold):
        res.id = -21 #torque hold to high
        return res
    handle.reqString = handle.reqString + predex + "9=" + str(req.torque_hold) 

    handle.stepnumber = handle.stepnumber + 1
    return res 
    
def TightenSignal(req):
     #setup
    ind = IndexForID(req.cid)
    res = on_externResponse()
    res.id = req.cid
    if(ind < 0):
        res.id = -22 #couldnt find id
        return res
    handle = InstanceList[ind]
    predex = "&p"+handle.SN()+"_"

    #set Programm Type
    handle.reqString = handle.reqString + predex + "block=4"
    handle.reqString = handle.reqString + predex + "num=1"

    #set max screwdriving time
    if(req.max_time > 65535):
        res.id = -23 #max time to high
        return res
    handle.reqString = handle.reqString + predex + "1=" + str(req.max_time)

    #set min angle
    if(req.angle_min > 9999999 or req.angle_min < 0):
        res.id = -24 #min angle invalid
        return res
    handle.reqString = handle.reqString + predex + "7=" + str(req.angle_min)   

    #set max angle
    if(req.angle_max > 9999999 or req.angle_max < 0):
        res.id = -25 #max angle invalid
        return res
    handle.reqString = handle.reqString + predex + "8=" + str(req.angle_max)   

    #set min torque
    if(req.torque_min > max_torque or req.torque_min < 0):
        res.id = -26 #torque min to high or negative
    handle.reqString = handle.reqString + predex + "3=" + "{:.1f}".format(req.torque_min) #TODO check if it must be rounded

    #set max torque
    if(req.torque_max > max_torque or req.torque_max < 0):
        res.id = -27 #torque cutoff to high or negative
    handle.reqString = handle.reqString + predex + "4=" + "{:.1f}".format(req.torque_max) #TODO check if it must be rounded

    #set screwdriver speed
    if(req.rpm > max_rpm or req.rpm < min_rpm):
        res.id = -28 #scredriver speed not legal
        return res
    handle.reqString = handle.reqString + predex + "10=" + str(req.rpm)

    #set final value creation
    if(req.create_val > 1 or req.create_val < 0):
        res.id = -29  #value creation invalid
        return res
    handle.reqString = handle.reqString + predex + "11=" + str(req.create_val)

    handle.stepnumber = handle.stepnumber + 1
    return res 

def LoosenSignal(req):
     #setup
    ind = IndexForID(req.cid)
    res = on_externResponse()
    res.id = req.cid
    if(ind < 0):
        res.id = -30 #couldnt find id
        return res
    handle = InstanceList[ind]
    predex = "&p"+handle.SN()+"_"

    #set Programm Type
    handle.reqString = handle.reqString + predex + "block=5"
    handle.reqString = handle.reqString + predex + "num=1"

    #set max screwdriving time
    if(req.max_time > 65535):
        res.id = -31 #max time to high
        return res
    handle.reqString = handle.reqString + predex + "1=" + str(req.max_time)

    #set min angle
    if(req.angle_min > 9999999 or req.angle_min < 0):
        res.id = -32 #min angle invalid
        return res
    handle.reqString = handle.reqString + predex + "7=" + str(req.angle_min)   

    #set max angle
    if(req.angle_max > 9999999 or req.angle_max < 0):
        res.id = -33 #max angle invalid
        return res
    handle.reqString = handle.reqString + predex + "8=" + str(req.angle_max)   

    #set min torque
    if(req.torque_min > max_torque or req.torque_min < 0):
        res.id = -34 #torque min to high or negative
    handle.reqString = handle.reqString + predex + "3=" + "{:.1f}".format(req.torque_min) #TODO check if it must be rounded

    #set max torque
    if(req.torque_max > max_torque or req.torque_max < 0):
        res.id = -35 #torque cutoff to high or negative
    handle.reqString = handle.reqString + predex + "4=" + "{:.1f}".format(req.torque_max) #TODO check if it must be rounded

    #set screwdriver speed
    if(req.rpm > max_rpm or req.rpm < min_rpm):
        res.id = -36 #scredriver speed not legal
        return res
    handle.reqString = handle.reqString + predex + "10=" + str(req.rpm)

    #set final value creation
    if(req.create_val > 1 or req.create_val < 0):
        res.id = -37  #value creation invalid
        return res
    handle.reqString = handle.reqString + predex + "11=" + str(req.create_val)

    handle.stepnumber = handle.stepnumber + 1
    return res 

def LoosenAngleSupervised(req):
    #setup
    ind = IndexForID(req.cid)
    res = loosen_angle_superResponse()
    res.id = req.cid
    if(ind < 0):
        res.id = -38 #couldnt find id
        return res
    handle = InstanceList[ind]
    predex = "&p"+handle.SN()+"_"

    #set Programm Type
    handle.reqString = handle.reqString + predex + "block=3"
    handle.reqString = handle.reqString + predex + "num=1"

    #set max screwdriving time
    if(req.max_time > 65535):
        res.id = -39 #max time to high
        return res
    handle.reqString = handle.reqString + predex + "1=" + str(req.max_time)

    #set shutoff angle
    if(req.angle_shutoff > 9999999 or req.angle_shutoff < 0):
        res.id = -40 #shutoff angle invalid
        return res
    handle.reqString = handle.reqString + predex + "6=" + str(req.angle_shutoff)

    #set min angle
    if(req.angle_min > 9999999 or req.angle_min < 0):
        res.id = -41 #min angle invalid
        return res
    handle.reqString = handle.reqString + predex + "7=" + str(req.angle_min)   

    #set max angle
    if(req.angle_max > 9999999 or req.angle_max < 0):
        res.id = -42 #max angle invalid
        return res
    handle.reqString = handle.reqString + predex + "8=" + str(req.angle_max)   

    #set min torque
    if(req.torque_min > max_torque or req.torque_min < 0):
        res.id = -43 #torque min to high or negative
    handle.reqString = handle.reqString + predex + "3=" + "{:.1f}".format(req.torque_min) #TODO check if it must be rounded

    #set max torque
    if(req.torque_max > max_torque or req.torque_max < 0):
        res.id = -44 #torque cutoff to high or negative
    handle.reqString = handle.reqString + predex + "4=" + "{:.1f}".format(req.torque_max) #TODO check if it must be rounded

    #set screwdriver speed
    if(req.rpm > max_rpm or req.rpm < min_rpm):
        res.id = -45 #scredriver speed not legal
        return res
    handle.reqString = handle.reqString + predex + "10=" + str(req.rpm)

    #set final value creation
    if(req.val_type > 1 or req.val_type < 0):
        res.id = -46  #value creation invalid
        return res
    handle.reqString = handle.reqString + predex + "11=" + str(req.val_type)

    handle.stepnumber = handle.stepnumber + 1
    return res 

def Setup(): #sets up all services etc
    rospy.init_node('deprag_cgi',anonymous=True)
    sCreate = rospy.Service('deprag_create',create_pgm,Create)
    sPub = rospy.Service('deprag_publish',publish,Publish)
    sTightTorqueUnsuo = rospy.Service('deprag_tighten_torque_unsupervised',tighten_torque_unsupervised,TightTorqueUnsup)
    sDisplayVal = rospy.Service('deprag_display_values',secondary_attribute,DispValues)
    sSaveVal = rospy.Service('deprag_save_values',secondary_attribute,SaveValues)
    sStatistics = rospy.Service('deprag_statistics',secondary_attribute,Statistics)
    sLoosenTorqueUnsup = rospy.Service('deprag_loosen_torque_unsupervised',loosen_torque_unsupervised,LoosenTorqueUnsup)
    sTightenSignal = rospy.Service('deprag_tighten_on_extern',on_extern,TightenSignal)
    sLoosenSignal = rospy.Service('deprag_loosen_on_extern',on_extern,LoosenSignal)
    sLoosenAngle = rospy.Service('deprag_loosen_angle_supervised',loosen_angle_super,LoosenAngleSupervised)

    rospy.spin()

if __name__ == "__main__":
    Setup()