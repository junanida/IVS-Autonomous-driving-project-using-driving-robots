
import requests

# server_ip = "192.168.201.115"  # 라즈베리파이 IP
#server_ip_changmin = "192.168.202.55" #창민 IP
#server_ip_doyup ="192.168.203.119" #도엽 
server_ip_junghwan="192.168.202.131" # 정환
# url_flag_sangwoo = f"http://{server_ip}:5000/flag"         # POST/GET 용
#url_flag_changmin = f"http://{server_ip_changmin}:8080/flag"         # POST/GET 용
#url_flag_doyup = f"http://{server_ip_doyup}:5000/flag"         # POST/GET 용
url_flag_junghwan = f"http://{server_ip_junghwan}:5000/flag"         # POST/GET 용

flag_value = 1; # 1을 주면 2초간 움직임. 0이면 정지 유지

response = requests.post(server_ip_junghwan, json={"value": flag_value})
response.raise_for_status()
print(response.json())

chang_min = 1;

#response = requests.post(url_flag_changmin, json={"value": chang_min})
#response.raise_for_status()
#print(response.json())

#jung_hwan=2;

#response = requests.post(url_flag_junghwan, json={"value": jung_hwan})
#response.raise_for_status()
#print(response.json())

#response = requests.post(url_flag_doyup, json={"value": flag_value})
#response.raise_for_status()
#print(response.json())
