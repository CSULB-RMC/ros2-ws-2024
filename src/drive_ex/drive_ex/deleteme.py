import can

bus = can.interface.Bus(interface='socketcan', channel='vcan0', bitrate='500000')


def signal_conversion(msg_data: int, bytes_range: int) -> list[int]:
        data: int = msg_data 
        temp_data: list[int] = []

        #make sure the direction is correct
        if data > 100:
            #carry for 2's comp
            c = 1

            # Forward msg correction:
            data -= 100
            # covert controller signal to proper range (1000-100000)
            data *= 1000

            #convert to byte array but also 2's compliment to reverse motor
            for i in range(bytes_range - 1, -1, -1):
                temp_data.append(255 - ((data >> (8*i)) & 0xff))

            for i in range(len(temp_data) - 1, - 1, -1):
                temp_data[i] += c 
                if temp_data[i] > 255:
                    temp_data[i] = 0
                else:
                    c = 0
                    break

        else:
            # covert controller signal to proper range (1000-100000)
            data *= 1000
    
            # convert signal to byte array
            for i in range(bytes_range - 1, -1, -1):
                temp_data.append((data >> (8*i)) & 0xff)
        
        return temp_data 

# checks if speed is different then previous message published

temp_data = signal_conversion(150, 4)  
# can message for right and left motor
can_msg_m1 = can.Message(
        arbitration_id = 16,
        data = temp_data, # place holder 
        is_extended_id = True
        )
# Changed so both motors are active on 16
#can_msg_m2 = can.Message(
#        arbitration_id = 15,
#        data = [32], # place holder
#        is_extended_id = True
#        )


# Send to Both Left Motors
bus.send(can_msg_m1)
bus.shutdown()