#!/usr/bin/env python

import sys

#print sys.argv
if len(sys.argv) != 4:
    print "Usage: template_file actions_file tidy_location"
    sys.exit(1)

templ = sys.argv[1]
actions = sys.argv[2]
t_loc = sys.argv[3]

ac_templ = ""
with open(templ, "r") as f:
    for line in f:
        ac_templ += line

ac_cont = ac_templ % {'tidy_loc': t_loc}

with open(actions, "w") as f:
    print >> f, ac_cont

