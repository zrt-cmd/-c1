#!/usr/bin/env python3
"""Search archived source sections and report original URLs and local files."""
import argparse
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('terms', nargs='+', help='literal search terms; all must occur in the same section')
    parser.add_argument('--course', default='', help='URL substring, e.g. exp004-slam-navigation')
    parser.add_argument('--limit', type=int, default=5)
    parser.add_argument('--full', action='store_true', help='print whole matched sections')
    args = parser.parse_args()
    terms = [t.casefold() for t in args.terms]
    hits = []
    for line in (ROOT / 'pages.jsonl').read_text(encoding='utf-8').splitlines():
        page = json.loads(line)
        if args.course not in page['url']:
            continue
        sections = page['sections'] or [{'id': '', 'text': (ROOT / page['text_path']).read_text(encoding='utf-8')}]
        for section in sections:
            body = section['text']
            folded = body.casefold()
            if not all(term in folded for term in terms):
                continue
            score = sum(folded.count(term) for term in terms) / math.sqrt(max(1, len(body)))
            if section['id']:
                score *= 2
            hits.append((score, page, section))
    hits.sort(key=lambda hit: (-hit[0], hit[1]['url'], hit[2]['id']))
    for _, page, section in hits[:max(0, args.limit)]:
        anchor = '#' + section['id'] if section['id'] else ''
        print(page['title'])
        print(page['url'] + anchor)
        print(ROOT / page['text_path'])
        body = section['text']
        if args.full:
            print(body)
        else:
            pos = min(body.casefold().find(term) for term in terms)
            print(body[max(0, pos - 120):pos + 500])
        print()
    print(f'Matched sections: {len(hits)}')


if __name__ == '__main__':
    main()
