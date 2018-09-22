#!/usr/bin/env python
# license removed for brevity

import rospy
import os.path
import os
import time
import filetype
from std_msgs.msg import String
import threading

voice_dict = {}
voice_material_dir = '/tmp/voice_materials/'

def SpeechPlay_GetFiles( term_dir ):
    term_list = os.listdir( term_dir )
    file_list = []

    for term in term_list:
        term_path = os.path.join( term_dir, term )
        if( os.path.isfile( term_path ) ):
            file_list.append( term_path )

    return file_list

def FileMonitor( ):
    voice_file_list = SpeechPlay_GetFiles( voice_material_dir )
    for file_path in voice_file_list:
        voice_material_key = os.path.splitext( os.path.basename( file_path ) )[ 0 ] 
        voice_dict[ voice_material_key ] = file_path

    timer = threading.Timer( 1, FileMonitor )
    timer.start()    

    print( "file monitor" )

def SpeechPlay_Callback( text ):
    global voice_publisher
    global tts_publisher
    voice_material_key = text.data[ 0: 20 ] 

    if( None != voice_dict.get( voice_material_key ) ):
        if( os.path.isfile( voice_dict.get( voice_material_key ) ) ):
            voice_publisher.publish( voice_dict.get( voice_material_key ) )
            print( "Send file to voice play node.")
        else:
            voice_dict.pop( voice_material_key )
            tts_publisher.publish( text )
            print( "Send text to tts node.")
    else:
        tts_publisher.publish( text )
        print( "Send text to tts node.")
                
            
if __name__ == '__main__':
    file_monitor = threading.Timer( 1, FileMonitor )
    file_monitor.start()

    try:
        rospy.init_node( 'speech_play_node', anonymous=True )
        rospy.Subscriber( 'SpeechPlay', String, SpeechPlay_Callback )
        voice_publisher = rospy.Publisher( 'VoicePlay', String, queue_size = 1 )
        tts_publisher = rospy.Publisher( 'tts', String, queue_size = 1 ) 

        voice_file_list = SpeechPlay_GetFiles( voice_material_dir )

        for file_path in voice_file_list:
            voice_material_key = os.path.splitext( os.path.basename( file_path ) )[ 0 ] 
            voice_dict[ voice_material_key ] = file_path

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
