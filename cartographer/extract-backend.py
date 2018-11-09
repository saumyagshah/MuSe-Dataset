#!/usr/bin/env python
# coding: utf-8

# In[1]:


import tf

lines_seen = set() # holds lines already seen
outfile = open('remove_duplicates.txt', 'w+')
for line in open('info_2.txt', 'r'):
    if line not in lines_seen: # not a duplicate
        outfile.write(line)
        lines_seen.add(line)
outfile.close()

max_node = float('-inf')
max_submap = float('-inf')
with open('remove_duplicates.txt', 'r') as f:
        while True:
            x = f.readline()
            if not x: break
            node_list = x.split()
#             print node_list[2]
            node_index, submap_index = node_list[0], node_list[1]
            node_index, submap_index = float(node_index), float(submap_index)
            if(node_index > max_node):
                max_node = node_index
            if(submap_index > max_submap):
                max_submap = submap_index
#                 temp = node_list[0]

with open('remove_duplicates.txt', 'r') as f:
    with open('isam_reordered.txt', 'w+') as g:
        while True:
            x = f.readline()
            if not x: break
            node_list = x.split()
            submap = int(node_list[1])
            node = int(node_list[0])
            translation_x, translation_y, translation_z  = float(node_list[2]), float(node_list[3]), float(node_list[4])
            quat_x, quat_y, quat_z, quat_w = float(node_list[5]), float(node_list[6]), float(node_list[7]), float(node_list[8])
            trans_weight = float(node_list[9])
            rot_weight = float(node_list[10])
#             g.write("EDGE2 %d %d "% (max_node + submap + 1, node) )
            g.write("EDGE2 %d %d "% (submap, node + max_submap + 1) )
            quaternion = (quat_x, quat_y, quat_z, quat_w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            yaw = euler[2]
#             g.write(translation[0] + " " translation[1] + " " + translation[2] + " ")
#             g.write(quat[0] + " " quat[1] + " " + quat[2] + "\n")
#             g.write("%f %f %f %f %f %f %f\n" % (translation_x, translation_y, translation_z, quat_x, quat_y, quat_z, quat_w))
            g.write("%f %f %f %f 0 0 %f 0 %f\n" % (translation_x, translation_y, yaw, trans_weight, trans_weight, rot_weight))

    


# In[ ]:




