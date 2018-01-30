See issues tab for more, or where relevant
Specific tasks:
 - Allow pressing enter to submit fields on UI
 - Profile websocket server and get rid of inefficiencies
 - Improve deploy script to:
 -- Only re-upload updated files
 -- Use proper options in deploy scripts rather than just positional arguments
 -- require fewer options when deploying
 - Remove unused code
 - Use mutexes that can be locked twice within the same thread (also, priority inversion).
 - Switch over to radians instead of degrees for lat/lon
 - Create some sort of CAN utilization measure

Controller/Planner:
 - Adaptive controller likes to let the sail luff too much, probably thinking it is getting some
   torque advantage by doing that. Don't do that.
 - Need to actually *use* the path we generate so that if we pass a
   tack point inbetween iterations, we can correct for it.
 - Both need tunable constants pulled out and handled properly
 - Planner needs to be able to receive obstacles cleanly
 - Planner needs to handle more sophisticated waypoint options.
 - Design things in a way such that the planner communicates a nominal
   and desired heading, so that the controller can, for instance, know
   whether to strictly hold a course, or if it should shift slightly
   up/downwind if changes in local conditions allow it.
 - Adjust polygon costs so that we have a clean gradient while a path is cutting through the obstacle
 - Fix theory_control.py so that we don't go faster upwind than on a beam reach
 - Movable ballast
 - Better debugging/UI for planner internals
