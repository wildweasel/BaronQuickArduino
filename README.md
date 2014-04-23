BaronQuickArduino
=================

Code for Android + Arduino + Baron4WD robot to replace Quickbot in Cousera Control of Mobile Robots class

Needs BaronQuickAndroid running on a ADK-compatible Android phone.

Listens for motor messages from the Android phone.  Motor messages tell motor(s) on each side of the bot what
direction and how much power.  Sends commands to the Arduino pins declared in the code to activate h-bridge driven
motor(s) accordingly.
