# This file to add the publish function for the v2x interface messages.

from v2x_msgs.msg import Recognition, Objects
import struct


class V2XMsgsPub:
    def __init__(self, connection_manager):
        super().__init__('v2x_msg_pub')

    def proc_v2x_msgs(self, r_data) :
        pass



