#!/usr/bin/env python3

from gz.msgs10.contacts_pb2 import Contacts
from gz.transport13 import Node
import time

class ContactSensor:
    def __init__(self):
        # Contact sensor topics
        self.topic_negx = "/contact_1"
        self.topic_posx = "/contact_2"
        self.topic_negy = "/contact_3"
        self.topic_posy = "/contact_4"

        # Contact flags
        self.negx = False
        self.posx = False
        self.negy = False
        self.posy = False

        self.totalcontacts = 0
        self.node = Node()

        # Subscriptions
        self.node.subscribe(Contacts, self.topic_negx, self.get_negx)
        self.node.subscribe(Contacts, self.topic_posx, self.get_posx)
        self.node.subscribe(Contacts, self.topic_negy, self.get_negy)
        self.node.subscribe(Contacts, self.topic_posy, self.get_posy)

        
        print("📡 Subscribed to all contact sensors.")

    def get_negx(self, msg: Contacts):
        #print("contact made")
        self.negx = len(msg.contact) > 0

    def get_posx(self, msg: Contacts):
        self.posx = len(msg.contact) > 0

    def get_negy(self, msg: Contacts):
        self.negy = len(msg.contact) > 0

    def get_posy(self, msg: Contacts):
        self.posy = len(msg.contact) > 0

    def get_total_contacts(self):
        total = 0
        if self.negx: total += 1
        if self.posx: total += 1
        if self.negy: total += 1
        if self.posy: total += 1
        self.posy = False  
        self.posx = False  
        self.negy = False  
        self.negx = False  
        self.totalcontacts = total
        return total

def main():
    contact_sensor = ContactSensor()

    try:
        while True:
            total = contact_sensor.get_total_contacts()
            print(f"Total contact sides: {total} | negx: {contact_sensor.negx}, posx: {contact_sensor.posx}, negy: {contact_sensor.negy}, posy: {contact_sensor.posy}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("👋 Shutting down contact sensor monitor.")

if __name__ == "__main__":    
    main()
