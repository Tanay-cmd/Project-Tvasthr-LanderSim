#!/usr/bin/env python3

from gz.msgs10.contacts_pb2 import Contacts
from gz.transport13 import Node
import time

class ContactSensor:
    def __init__(self):
        # Contact sensor topics
        self.topic_negx = "/world/moon_flat_world/model/apollo_lander/link/lander_link/sensor/contact_sensor_negx/contact"
        self.topic_posx = "/world/moon_flat_world/model/apollo_lander/link/lander_link/sensor/contact_sensor_posx/contact"
        self.topic_negy = "/world/moon_flat_world/model/apollo_lander/link/lander_link/sensor/contact_sensor_negy/contact"
        self.topic_posy = "/world/moon_flat_world/model/apollo_lander/link/lander_link/sensor/contact_sensor_posy/contact"

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
