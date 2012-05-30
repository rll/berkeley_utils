#!/usr/bin/env python                                                                                                                                        
"""
Contains phrases for brett to say
"""
import random
import subprocess

BRETT_TALK = False

# Hi Molly and Heather, my name is...

#talk_greet = ["Hi! My name is Brett. ... ... I love organizing laundry."]
#talk_greet = ["Hi Molly and Heatherrr! My name is Brett. I was built at Willow Garage. And my artificial intelligence was programmed at U C Berkeley. I love organizing laundry. "]
talk_greet = ["Hi Molly and Heatherrr! My name is Brett. I love organizing laundry. "]
#talk_initialize = ["I will fold this towell for you"]
talk_initialize = ["In particular, I take great pleasure in folding towels"]

#talk_pickupclump = ["First, I need to find a way to spread out this towell"] 
talk_pickupclump = ["First, I need to find a way to spread out this towel and make things less hektikk"] 
talk_pose1 = ["I am pose ing for photographs"]
talk_pose2 = ["Computation complete."]
talk_drag1 = ["Dragging the towell, will help me find a corn ner"]
talk_drag2 = ["Now I am trying to find the other corn ner", "Find, ding another corn ner"]
#talk_triangles = ["The towell is now in either of too states. Triangle left or triangle right"]
talk_triangles = ["Proh-sehseen."]
talk_spreadout = ["Spread ding out the towell"]

#talk_done = ["Here you go","Go Bears", "I am done"]

talk_done = ["There we go, I hope I lived up to NEAT method's standards. "]
             #"Would you like to pose for a picture with me?"]

talk_reset = ["Oops. I will have to try again."]



def pr2_say(phrase_list):
    if not BRETT_TALK:
        return

    index = int(random.random()*len(phrase_list))
    process = subprocess.Popen(['festival', '--tts'],
                           stdin=subprocess.PIPE,
                           stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT)
    stdout_value, stderr_value = process.communicate(phrase_list[index])
