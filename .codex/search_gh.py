import urllib.request, json, ssl

queries = [
    "zebra+crossing+smart+car+vision",
    "nxp+smart+car+vision+zebra",
    "intelligent+car+vision+zebra",
]
ssl_ctx = ssl.create_default_context()
for q in queries:
    url = f"https://api.github.com/search/repositories?q={q}&sort=stars&order=desc&per_page=5"
    try:
        req = urllib.request.Request(url, headers={"Accept": "application/vnd.github.v3+json", "User-Agent": "codex-agent"})
        with urllib.request.urlopen(req, timeout=15, context=ssl_ctx) as resp:
            data = json.loads(resp.read())
            print(f"=== {q} ===")
            for item in data.get("items", []):
                desc = item.get("description","") or "N/A"
                print(f'{item["full_name"]} | stars={item["stargazers_count"]} | {desc}')
            print()
    except Exception as e:
        print(f"{q}: {e}")