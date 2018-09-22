#!/usr/bin/env python
# license removed for brevity

import rospy
import os.path
import os
import time
import filetype
from std_msgs.msg import String
import threading
import hashlib
from xfyun_tts.srv import *
import shutil

voice_dict = {}
lock_of_voice_dict = threading.Lock()

voice_material_dir = os.environ[ 'HOME' ] + '/Music/voice_materials/'

def SpeechPlay_GetFiles( term_dir ):
    term_list = os.listdir( voice_material_dir + 'music/' )
    file_list = []

    for term in term_list:
        term_path = os.path.join( term_dir + 'music/', term )
        if( os.path.isfile( term_path ) ):
            file_list.append( term_path )
    return file_list

def SpeechPlay_TTS( text ):
    try:
        speech_synthesis = rospy.ServiceProxy( 'SpeechSynthesis', SpeechSynthesis )
        synthesis_file = speech_synthesis( text.data )
        return synthesis_file.file_path
    except rospy.ServiceException, e:
        print( "Service call failed: %s" % e )
    return "None"

def SpeechPlay_Callback( text ):
    global voice_publisher
    global tts_publisher
    global voice_dict

    voice_material_key = text.data

    lock_of_voice_dict.acquire()
    local_dict = voice_dict
    lock_of_voice_dict.release()

    speech_file_name = local_dict.get( voice_material_key )

    if( None != speech_file_name ):
        # Try the map way 
        voice_publisher.publish( speech_file_name )
        print( "Play specific music.")
    else:
        # Try the hash way
        speech_file_name = voice_material_dir + "speech/" + hashlib.md5( text.data ).hexdigest() + ".wav"

        if( os.path.isfile( speech_file_name ) ):
            voice_publisher.publish( speech_file_name )
            print( "Play speech in base.")
        else:
            print( "Call tts service.")
            synthesis_file = SpeechPlay_TTS( text )

            if( os.path.isfile( synthesis_file ) ):
                speech_file_name = voice_material_dir + "speech/" + hashlib.md5( text.data ).hexdigest() + ".wav"
                shutil.copyfile( synthesis_file, speech_file_name )

                voice_publisher.publish( speech_file_name )
                print( "Play synthesis voice.")
            else:
                print( "TTS failed" )
            
def SpeechPlay_FileMonitor( ):
    global voice_dict

    while( True ):
        local_dict = {}
        voice_file_list = SpeechPlay_GetFiles( voice_material_dir )

        for file_path in voice_file_list:
            voice_material_key = os.path.splitext( os.path.basename( file_path ) )[ 0 ] 
            local_dict[ voice_material_key ] = file_path

        lock_of_voice_dict.acquire()
        voice_dict = local_dict
        lock_of_voice_dict.release()

        time.sleep( 3 )

if __name__ == '__main__':

    if( False == os.path.exists( voice_material_dir ) ):
        os.mkdir( voice_material_dir )

    if( False == os.path.exists( voice_material_dir + 'music/' ) ):
        os.mkdir( voice_material_dir + 'music/' )
    
    if( False == os.path.exists( voice_material_dir + 'speech/' ) ):
        os.mkdir( voice_material_dir + 'speech/' )

    try:
        rospy.wait_for_service( 'SpeechSynthesis', 3 )
    except rospy.ROSException, e:
        print( "TTS service is closed" )

    file_monitor = threading.Thread( target = SpeechPlay_FileMonitor, name = "FileMonitor" )
    file_monitor.setDaemon( True )
    file_monitor.start()

    rospy.init_node( 'speech_play_node', anonymous=True )
    rospy.Subscriber( 'SpeechPlay', String, SpeechPlay_Callback )
    voice_publisher = rospy.Publisher( 'VoicePlay', String, queue_size = 1 )
    tts_publisher = rospy.Publisher( 'xfyun_tts', String, queue_size = 1 ) 

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
