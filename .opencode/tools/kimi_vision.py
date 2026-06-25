#!/usr/bin/env python3
"""Vision - 用 Kimi 或 DeepSeek 读取图片内容"""
import sys, json, base64, os, urllib.request

KM_KEY = 'sk-n4PsIi2TVvuP3lxqVZSreJNG8imeqHWGPO4V5kSqAACOlQnj'
KM_URL = 'https://api.moonshot.cn/v1/chat/completions'
KM_MODEL = 'kimi-k2.6'

def encode_image(path):
    with open(path, 'rb') as f:
        return base64.b64encode(f.read()).decode('utf-8')

def ask_kimi(image_path, prompt):
    b64 = encode_image(image_path)
    ext = os.path.splitext(image_path)[1].lower().lstrip('.')
    mime_map = {'jpg':'image/jpeg','jpeg':'image/jpeg','png':'image/png',
                'gif':'image/gif','webp':'image/webp','bmp':'image/bmp'}
    mime = mime_map.get(ext, 'image/jpeg')
    payload = json.dumps({
        'model': KM_MODEL,
        'messages': [{'role':'user','content':[
            {'type':'text','text':prompt},
            {'type':'image_url','image_url':{'url':f'data:{mime};base64,{b64}'}}
        ]}],
        'temperature':1,'max_tokens':2048,
    }).encode()
    req = urllib.request.Request(KM_URL, data=payload, headers={
        'Content-Type':'application/json','Authorization':f'Bearer {KM_KEY}'})
    try:
        resp = urllib.request.urlopen(req, timeout=60)
        result = json.loads(resp.read())
        return result['choices'][0]['message']['content']
    except urllib.error.HTTPError as e:
        return f'[Kimi Error] HTTP {e.code}: {e.read().decode()}'
    except Exception as e:
        return f'[Error] {e}'

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: kimi_vision.py <image_path> [prompt]'); sys.exit(1)
    print(ask_kimi(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else '请详细描述这张图片里的内容'))
