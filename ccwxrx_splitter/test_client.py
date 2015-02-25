from multiprocessing.connection import Client

address = ('127.0.0.1', 5772)
conn = Client(address)
conn.send(1)
while True:
    message = conn.recv()
    print message