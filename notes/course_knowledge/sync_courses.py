#!/usr/bin/env python3
"""Archive linked course pages with source URLs, hashes and retrieval coverage."""
import concurrent.futures
import hashlib
import json
import re
import time
import urllib.error
import urllib.parse
import urllib.request
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from bs4 import BeautifulSoup

ROOT = Path(__file__).resolve().parent
SITE = 'https://aibittek.github.io/course_generate_agent/'
START = SITE + 'courses/index.html'
BASE_PATH = urllib.parse.urlsplit(SITE).path


def normalize(url, base=START):
    p = urllib.parse.urlsplit(urllib.parse.urljoin(base, url))
    if p.scheme not in ('http', 'https'):
        return None
    path = p.path
    if path.endswith('/'):
        path += 'index.html'
    return urllib.parse.urlunsplit((p.scheme, p.netloc, path, '', ''))


def internal(url):
    p = urllib.parse.urlsplit(url)
    return p.netloc == 'aibittek.github.io' and p.path.startswith(BASE_PATH)


def local_path(url):
    return urllib.parse.unquote(urllib.parse.urlsplit(url).path[len(BASE_PATH):])


def get(url):
    for attempt in range(3):
        try:
            request_url = urllib.parse.quote(url, safe=':/%?=&')
            req = urllib.request.Request(request_url, headers={'User-Agent': 'CourseKnowledgeArchive/1.0'})
            with urllib.request.urlopen(req, timeout=25) as response:
                data = response.read()
                return data, response.geturl(), response.headers.get('Content-Type', ''), None
        except Exception as exc:
            if isinstance(exc, urllib.error.HTTPError) and exc.code in (400, 401, 403, 404, 410):
                return None, url, '', str(exc)
            if attempt == 2:
                return None, url, '', str(exc)
            time.sleep(attempt + 1)


def extract(url, data):
    soup = BeautifulSoup(data, 'html.parser')
    title = soup.title.get_text(' ', strip=True) if soup.title else url
    links = []
    for tag in soup.find_all(['a', 'iframe', 'script', 'img', 'link', 'source', 'video', 'audio']):
        attr = 'href' if tag.name in ('a', 'link') else 'src'
        raw = tag.get(attr)
        if raw:
            target = normalize(raw, url)
            if target:
                links.append({'url': target, 'kind': tag.name, 'label': tag.get_text(' ', strip=True)[:120]})
    sections = []
    for sec in soup.select('section[id]'):
        copy = BeautifulSoup(str(sec), 'html.parser')
        for t in copy(['script', 'style', 'nav']):
            t.decompose()
        sections.append({'id': sec['id'], 'text': copy.get_text('\n', strip=True)})
    code = [x.get_text('\n', strip=False) for x in soup.find_all('pre')]
    for tag in soup(['script', 'style', 'nav']):
        tag.decompose()
    text = soup.get_text('\n', strip=True)
    return title, text, sections, code, links


def run():
    now = datetime.now(timezone.utc).isoformat()
    manifest, references, pages = {}, {}, []
    pending = {START}
    with concurrent.futures.ThreadPoolExecutor(max_workers=5) as pool:
        while pending:
            batch = sorted(pending - manifest.keys())
            pending.clear()
            if not batch:
                break
            for url, result in zip(batch, pool.map(get, batch)):
                data, final_url, content_type, error = result
                row = {'url': url, 'fetched_at': now, 'final_url': final_url, 'content_type': content_type}
                if error:
                    row.update(status='failed', error=error)
                else:
                    rel = local_path(url)
                    dest = ROOT / 'html' / rel
                    dest.parent.mkdir(parents=True, exist_ok=True)
                    dest.write_bytes(data)
                    title, text, sections, code, links = extract(url, data)
                    text_rel = str(Path('text') / Path(rel).with_suffix('.txt'))
                    target = ROOT / text_rel
                    target.parent.mkdir(parents=True, exist_ok=True)
                    target.write_text('SOURCE: ' + url + '\nTITLE: ' + title + '\nFETCHED: ' + now + '\n\n' + text, encoding='utf-8')
                    row.update(status='ok', title=title, sha256=hashlib.sha256(data).hexdigest(), bytes=len(data), text_chars=len(text), text_path=text_rel, sections=[s['id'] for s in sections])
                    pages.append({'url': url, 'title': title, 'text_path': text_rel, 'sections': sections, 'code_blocks': code})
                    for link in links:
                        refs = references.setdefault(link['url'], {'url': link['url'], 'kinds': [], 'from': []})
                        if link['kind'] not in refs['kinds']:
                            refs['kinds'].append(link['kind'])
                        if url not in refs['from']:
                            refs['from'].append(url)
                        if internal(link['url']) and urllib.parse.urlsplit(link['url']).path.endswith('.html'):
                            pending.add(link['url'])
                manifest[url] = row
            print('Pages',len(manifest),'next',len(pending-manifest.keys()), flush=True)
    assets = []
    for u, r in references.items():
        path = urllib.parse.urlsplit(u).path.lower()
        if internal(u) and not path.endswith('.html'):
            assets.append(u)
    # Archive course-linked local assets as well, including interactive scripts and PDFs.
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        for url, result in zip(sorted(assets), pool.map(get, sorted(assets))):
            data, final_url, ct, error = result
            row = {'url': url, 'fetched_at': now, 'final_url': final_url, 'content_type': ct}
            if error:
                row.update(status='failed', error=error)
            else:
                rel = Path('assets') / local_path(url)
                dest = ROOT / rel
                dest.parent.mkdir(parents=True, exist_ok=True)
                dest.write_bytes(data)
                row.update(status='ok', bytes=len(data), sha256=hashlib.sha256(data).hexdigest(), path=str(rel))
            manifest[url] = row
    (ROOT/'manifest.json').write_text(json.dumps(list(manifest.values()),ensure_ascii=False,indent=2),encoding='utf-8')
    (ROOT/'references.json').write_text(json.dumps(list(references.values()),ensure_ascii=False,indent=2),encoding='utf-8')
    (ROOT/'pages.jsonl').write_text(''.join(json.dumps(p,ensure_ascii=False)+'\n' for p in pages),encoding='utf-8')
    lines=['# 页面索引','',f'入口 {START}', f'抓取时间（UTC） {now}', '', '| 页面 | 原文 | 本地正文 | 字符数 |','|---|---|---|---|']
    for row in sorted(manifest.values(), key=lambda r:r['url']):
        if row.get('text_path'):
            lines.append(f"| {row['title'].replace('|','/')} | [来源]({row['url']}) | [正文]({row['text_path']}) | {row['text_chars']} |")
    (ROOT/'INDEX.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    counts=Counter(r['status'] for r in manifest.values())
    print(json.dumps({'pages':len(pages),'assets':len(assets),'status':dict(counts),'chars':sum(r.get('text_chars',0) for r in manifest.values()),'failed':[r for r in manifest.values() if r['status']=='failed']},ensure_ascii=False),flush=True)

if __name__ == '__main__':
    run()
