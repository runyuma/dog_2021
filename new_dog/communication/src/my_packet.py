

class MyPacket:
    def __init__(self, ReceiverIndex, Output):
        self.Index = ReceiverIndex
        self.Data = []
        self.Ports = {}
        self.Port = 0
        self.Len = 0
        self.Len_Cur = 0
        self.CheckSum = 0
        self.State = 0
        self.Output = Output
        
    def Input(self, ByteStream):
        #print('In:', ByteStream)
        #print('Ls:', list(ByteStream))
        for rx in ByteStream:
            #print(self.State, self.Len_Cur)
            if self.State == 0:
                if rx == 0x7f:
                    self.State = 1
            elif self.State == 1:
                if rx & 0x0f == self.Index:
                    self.Port = rx >> 4
                    self.State = 2
                else:
                    self.State = 0
            elif self.State == 2:
                self.Len = rx
                self.Data = []
                self.Len_Cur = 0
                self.State = 3
                self.CheckSum = 0
            elif self.State == 3:
                if self.Len_Cur < self.Len:
                    self.Data.append(rx)
                    self.CheckSum += rx
                    self.Len_Cur += 1
                else:
                    if self.CheckSum % 256 == rx:
                        #print('Port {} in'.format(self.Port))
                        PortFunc = self.Ports.get(self.Port)
                        if PortFunc != None:
                            PortFunc(bytes(self.Data))
                    self.State = 0
                    
    def Send(self, Index, Port, Data):
        DataLen = len(Data)        
        Checksum = sum(Data) % 256
        Data_s = bytes([0x7f, Index + Port * 16, DataLen]) + bytes(Data) + bytes([Checksum])
        #print('Out:', list(Data_s))
        self.Output(Data_s)
    
    def SetCallback(self, Port, Callback):
        self.Ports[Port] = Callback

if __name__ == '__main__':
    def out(x):
        print(x)
    p = MyPacket(1, out)
    p.Input(bytes([127, 17, 4, 12, 0, 13, 0, 25]))
    p.Input(bytes([127, 1, 2, 111, 107, 218]))
    p.Input(bytes([127, 33, 9, 0, 0, 2, 0, 3, 0, 8, 1, 68, 82]))
