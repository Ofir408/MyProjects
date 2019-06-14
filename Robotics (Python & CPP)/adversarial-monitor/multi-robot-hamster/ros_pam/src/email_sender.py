#!/usr/bin/env python

import os
import smtplib
import ssl
import sys

from definitions import Definitions


class EmailSender:
    PACKAGE_NAME = "log_files"
    FILE_NAME = "log.txt"
    BASE_PATH = "/src/logger/" + PACKAGE_NAME  # will concatenate between ../catkin_ws to BASE_PATH

    def __init__(self):
        self.file_absolute_path = EmailSender.__get_catkin_ws_path() + self.BASE_PATH + "/" + self.FILE_NAME
        self.receiver_email = Definitions.email_receiver

    @staticmethod
    def __get_catkin_ws_path():
        str_to_search = "catkin_ws"
        current_path = os.getcwd()
        print "os.getcwd() is: " + current_path
        if str_to_search not in current_path:
            str_to_search = "hamster_ws"  # lab environment.
        print " (1) str_to_search is: " + str_to_search
        first_catkin_ws_inx = current_path.find(str_to_search)
        print "first_catkin_ws_inx is: " + str(first_catkin_ws_inx)
        str_to_return = current_path[0: first_catkin_ws_inx + len(str_to_search)]
        if first_catkin_ws_inx < 0:
            str_to_return = "/home/pi/hamster_ws"
        print "__get_catkin_ws_path returned: " + str_to_return
        return str_to_return

    def __get_msg(self):
        try:
            print "file_absolute_path is: " + self.file_absolute_path
            with open(self.file_absolute_path, "r") as f:
                return str(f.read())
        except (OSError, IOError):
            print "can't send an email"

    def send_email(self):
        print "self.__get_msg() is: " + str(self.__get_msg())
        port = 586  # starttls
        smtp_server = "smtp.gmail.com"
        sender_email = "adversarial.robot.monitor@gmail.com"
        password = "adversarial2019"
        message = (
                "Subject: NEW Adversarial Monitor Report\n"
                "    \n"
                + self.__get_msg())

        context = ssl.create_default_context()
        server = smtplib.SMTP_SSL(smtp_server, port)
        server.starttls(context=context)
        server.login(sender_email, password)
        server.sendmail(sender_email, self.receiver_email, message)
        print "Email sent successfully"


if __name__ == '__main__':
    print "email_sender started"
    rospy.init_node("sender", anonymous=True)
    # check if need to send an email.
    es = EmailSender()
    es.send_email()

