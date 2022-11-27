
from __future__ import unicode_literals
import yt_dlp as ytd
import sys
import pandas as pd
'''
df = pd.read_csv('jakis.csv')
indata =[]

for x in range(len(df)):
    indata = [df['link'][x]]
    if indata[0] == 'x':
        sys.exit()

    ydl_opts = {
            'format': '22/best',
            'outtmpl': f"C:/Users/Black/Desktop/Spongebob/S04E{df['ID'][x]}.mp4",
                #'postprocessors': [{
                    #'key': 'FFmpegExtractAudio',
                    #'preferredcodec': 'mp4',
                    #'preferredquality': '192',
                #}],
        }
    with youtube_dl.YoutubeDL(ydl_opts) as ydl:
        ydl.download(indata)
'''
while True:
    data = [input('link: ')]
    if not data:
        continue

    ydl_opts = {
            'format': 'bestaudio/best',
            'outtmpl': f"C:/Users/Black/Desktop/%(title)s.wav",
            
        }

    with ytd.YoutubeDL(ydl_opts) as ydl:
        ydl.download(data)



