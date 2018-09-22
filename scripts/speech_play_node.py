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

voice_dict = {}
lock_of_voice_dict = threading.Lock()

voice_material_dir = os.environ[ 'HOME' ] + '/Music/voice_materials/'

print( voice_material_dir )


def SpeechPlay_GetFiles( term_dir ):
    term_list = os.listdir( voice_material_dir + 'music/' )
    file_list = []

    for term in term_list:
        term_path = os.path.join( term_dir + 'music/', term )
        if( os.path.isfile( term_path ) ):
            file_list.append( term_path )
    return file_list


def SpeechPlay_Callback( text ):
    global voice_publisher
    global tts_publisher

    voice_material_key = text.data

    lock_of_voice_dict.acquire()
    local_dict = voice_dict
    lock_of_voice_dict.release()

    if( None != local_dict.get( voice_material_key ) ):
        # Try the map way 
        if( os.path.isfile( local_dict.get( voice_material_key ) ) ):
            voice_publisher.publish( local_dict.get( voice_material_key ) )
            print( "Send file to voice play node.")
        else:
            local_dict.pop( voice_material_key )
            tts_publisher.publish( text )
            print( "Send text to tts node.")
    else:
        # Try the hash way
        hash_md5 = hashlib.md5( text.data ).hexdigest()
        speech_file_name = voice_material_dir + "speech/" + hash_md5 + ".wav"

        if( os.path.isfile( speech_file_name ) ):
            voice_publisher.publish( speech_file_name )
            print( "Send file to voice play node.")
        else:
            tts_publisher.publish( text )
            print( "Send text to tts node.")
            
def SpeechPlay_FileMonitor( ):
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
