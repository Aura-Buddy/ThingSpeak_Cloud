"""
Mobile App for Senior Design IOT Alert System Project
Creator: Aura Teasley
For: UpRight Device
Team: Brandon Tiner, Levi Newton, Aura Teasley
"""

import bluetooth
import kivy
import time
from kivy.app import App
from kivy.uix.button import Button
from kivy.uix.gridlayout import GridLayout
from kivy.uix.textinput import TextInput
import thingspeak

class UpRightApp(App):

    def bluetooth(self):
        target_address = "FC:F5:C4:01:19:3A"  #must adjust for your microcontroller
        nearby_devices = bluetooth.discover_devices()

        print("addresses found: \n")
        for bdaddr in nearby_devices:
            if bdaddr == target_address:
                print("{}".format(target_name))
                break

        if bdaddr is not None:
            client_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            port = 1
            client_sock.connect((target_address, port))
            print("Accepted connection from: {}".format(target_address))

            message = [self.NetworkName.text, self.NetworkPassword.text, self.ChannelID.text, self.ChannelWriteKey.text]
            new_message = self.NetworkName.text+"+"+self.NetworkPassword.text+"+"+self.ChannelID.text+"+"+self.ChannelWriteKey.text+" #"
            """
            for i in range(4):
                print("Sending {}".format(message[i]))
                encoded_message = message[i].encode('utf-8')
                client_sock.send(encoded_message)
                time.sleep(2)
            """
            print("sending new message: {}".format(new_message))
            encoded_message = new_message.encode('utf-8')
            client_sock.send(encoded_message)
                
            client_sock.close()

        else:
            print("could not find target bluetooth device nearby")


    def build(self):
        layout = GridLayout(cols = 1)
        self.NetworkName = TextInput(text = "Enter Network Name here")
        self.NetworkPassword = TextInput(text = 'Enter Network Password here')
        self.ChannelID = TextInput(text = "Enter Channel ID here")
        self.ChannelWriteKey = TextInput(text = "Enter Channel Write Key here")
        self.ChannelReadKey = TextInput(text = "Enter Channel Read Key here")
        submit = Button(text = 'Submit', on_press=self.submit)
        layout.add_widget(self.NetworkName)
        layout.add_widget(self.NetworkPassword)
        layout.add_widget(self.ChannelID)
        layout.add_widget(self.ChannelWriteKey)
        layout.add_widget(self.ChannelReadKey)
        layout.add_widget(submit)
        return layout

    def submit(self,obj):
        print('Network Name: ' + self.NetworkName.text)
        print('Network Password: ' + self.NetworkPassword.text)
        print('Channel ID: ' + self.ChannelID.text)
        print('Channel Write Key: ' + self.ChannelWriteKey.text)
        print('Channel Read Key: ' + self.ChannelReadKey.text)
        NetworkName = self.NetworkName.text
        NetworkPassword = self.NetworkPassword.text
        ChannelID = self.ChannelID.text
        ChannelWriteKey = self.ChannelWriteKey.text
        ChannelReadKey = self.ChannelReadKey.text
        self.bluetooth()

if __name__ == '__main__':
    UpRightApp().run()


