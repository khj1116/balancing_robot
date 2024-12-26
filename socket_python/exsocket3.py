import socket

# 서버의 IP 주소와 포트 번호
server_address = ('서버_IP_주소', 10000)  # 서버의 실제 IP 주소로 변경해야 함

# 소켓 생성 및 서버에 연결
client_socket = socket.socket()
client_socket.connect(server_address)

# 사용자로부터 p32 값 입력받기
try:
    value = int(input("값을 입력하세요: "))  # 정수로 변환
    # 입력값 전송
    client_socket.send(str(value).encode())  # 문자열로 변환 후 바이트로 인코딩하여 전송
    print(f"서버로 전송된 값: {value}")

    # 서버로부터 응답 받기
    response = client_socket.recv(1024)  # 최대 1024바이트 수신
    print(f"서버로부터 받은 응답: {response.decode()}")

except ValueError:
    print("잘못된 입력입니다. 정수를 입력하세요.")

finally:
    # 연결 종료
    client_socket.close()











# #클라이언트코드

# import socket

# # 서버의 IP 주소와 포트 번호
# server_address = ('서버_IP_주소', 10000)

# # 소켓 생성 및 서버에 연결
# client_socket = socket.socket()
# client_socket.connect(server_address)

# # 전송할 메시지
# message = "안녕하세요"
# client_socket.send(message.encode())  # 메시지를 서버에 전송

# # 서버로부터 응답 받기
# response = client_socket.recv(1024)  # 최대 1024바이트 수신
# print(f"서버로부터 받은 응답: {response.decode()}")

# # 연결 종료
# client_socket.close()
