from random import random
import requests
import random

data = [random.randint(0,255) for i in range(32)]

res = requests.post('http://scargo.fr', files = {'data': str(data)})

print(data)
print(f'Response Code : {res.status_code}')