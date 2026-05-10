# Alert message
# Header timestamp represents time at witch alert was raised. 

std_msgs/Header header
uint8 severity # info, warn, abort, special (responds to acks)
string info_msg # human readible message that can be directly displayed on guis
uint32 nuid # non-unique id for internal use, composed of alert name and index. Also sorted to each node. 
string source_node_name
# uint32 nuid # non unique id. (ie, unique-enough id)
