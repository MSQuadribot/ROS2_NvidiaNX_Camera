import os
import time
import array
import asyncio
import httpx
import glob

async def post(liste):
    async with httpx.AsyncClient() as client: 
        for i in liste:
            start = time.time()
            if os.path.isfile(i):
                print('starting...')
                resp = await client.post('https://scargo.fr',files = {'Image': open(i,'rb')})
                print(f'Response Code : {resp.status_code}')
            print("Time --- %s seconds ---" % (time.time() - start))
            print(i)

def main():

        image_list = []
        for i in range(1,26):
            image_list.append(f'/home/qb/Desktop/dev_ws/src/camera/camera/Image/Image{i}.jpg')
        
        print(image_list)

        loop = asyncio.get_event_loop()
        while True :
            loop.run_until_complete(post(image_list))