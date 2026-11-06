"""Stage B3: rudrac fmt and the language server."""
import glob
import json
import os
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from rudrac import check, parse
from rudrac.fmt import format_checked, format_source
from rudrac.lsp import diagnostics, format_edits, hover, symbols
from rudrac.syntax import lex
from rudrac.util import RudraError
from rudrac import ir as I

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
EX = sorted(glob.glob(os.path.join(ROOT, "examples", "**", "*.rudra"), recursive=True))

MESSY = '''node   t target nano{
output  l:digital(pin 5,safe false)   
  func f(a:int,b:int)->int{
return a+b*-2 }
		on GO(v){ l=not (v>=1)and f( v ,-3)>0
if v==1{l = true}else{ l=false }
 for i in 0..8 { send "A",i }
    }


   // standalone comment
   heartbeat   3s   # hb
}'''


def toks(src):
    return [(t.kind, t.text, t.value) for t in lex(src)]


class FormatterTests(unittest.TestCase):
    def test_examples_are_already_tidy_and_idempotent(self):
        self.assertGreater(len(EX), 8)
        for f in EX:
            s = open(f).read()
            a = format_checked(s)
            self.assertEqual(a, s, f)
            self.assertEqual(format_checked(a), a, f)

    def test_messy_input(self):
        out = format_checked(MESSY)
        self.assertEqual(format_checked(out), out)
        self.assertEqual(toks(out), toks(MESSY))
        self.assertIn("  func f(a: int, b: int) -> int {", out)
        self.assertIn("return a + b * -2 }", out)
        self.assertIn("for i in 0..8 { send \"A\", i }", out)
        self.assertNotIn("\t", out)
        self.assertNotIn("\n\n\n", out)
        self.assertTrue(out.endswith("}\n"))
        self.assertTrue(all(l == l.rstrip() for l in out.split("\n")))

    def test_comments_survive(self):
        out = format_checked(MESSY)
        self.assertIn("// standalone comment", out)
        self.assertIn("# hb", out)
        # a comment before a closing brace sits at the brace's inner level
        src = "node t target nano {\nheartbeat 3s\n// last\n}\n"
        self.assertIn("\n  // last\n", format_checked(src))

    def test_meaning_unchanged_ir(self):
        """Re-indented examples lower to the very same IR as the originals."""
        for f in EX:
            s = open(f).read()
            if "import" in s:
                continue
            mess = "\n".join(l.strip() for l in s.split("\n"))
            a = I.dump(check(parse(s, f, {})))
            b = I.dump(check(parse(format_checked(mess), f, {})))
            self.assertEqual(a, b, f)

    def test_unary_minus_and_calls(self):
        out = format_checked("node t target nano {\non X(v) { v = -v + f(-1) - 2 * (-3) }\nheartbeat 1s\n}\n")
        self.assertIn("v = -v + f(-1) - 2 * (-3)", out)

    def test_bad_source_is_refused(self):
        with self.assertRaises(RudraError):
            format_checked('node t target nano { on X { send "oops } }')

    def test_cli_check_and_write(self):
        d = tempfile.mkdtemp()
        p = os.path.join(d, "m.rudra")
        open(p, "w").write(MESSY)
        env = dict(os.environ, PYTHONPATH=ROOT)
        r = subprocess.run([sys.executable, "-m", "rudrac", "fmt", "--check", p], capture_output=True, text=True, env=env)
        self.assertEqual(r.returncode, 1)
        r = subprocess.run([sys.executable, "-m", "rudrac", "fmt", p], capture_output=True, text=True, env=env)
        self.assertEqual(r.returncode, 0, r.stderr)
        r = subprocess.run([sys.executable, "-m", "rudrac", "fmt", "--check", p], capture_output=True, text=True, env=env)
        self.assertEqual(r.returncode, 0)

    def test_every_example_after_mess_formats_back_to_same_tokens(self):
        for f in EX:
            s = open(f).read()
            mess = "\n".join(l.strip() for l in s.split("\n"))          # lose all indentation
            out = format_checked(mess)
            self.assertEqual(toks(out), toks(s), f)
            self.assertEqual(format_checked(out), out, f)


class LspFunctionTests(unittest.TestCase):
    GOOD = "node t target nano {\n  output l: digital(pin 5, safe false)\n  on GO { l = true }\n  heartbeat 3s\n}\n"

    def test_clean_file_has_no_diagnostics(self):
        self.assertEqual(diagnostics(self.GOOD), [])

    def test_error_position(self):
        bad = self.GOOD.replace("l = true", "lamp = true")
        d = [x for x in diagnostics(bad) if x["severity"] == 1]
        self.assertEqual(len(d), 1)
        self.assertEqual(d[0]["range"]["start"]["line"], 2)
        self.assertIn("lamp", d[0]["message"])
        self.assertEqual(d[0]["range"]["end"]["character"] - d[0]["range"]["start"]["character"], 4)

    def test_syntax_error(self):
        d = diagnostics("node t target nano { on GO { l = } }")
        self.assertEqual(len(d), 1)
        self.assertEqual(d[0]["severity"], 1)

    def test_warning_severity(self):
        d = diagnostics(self.GOOD.replace("  on GO { l = true }\n", "  func spare() -> int { return 1 }\n"))
        self.assertTrue(d and all(x["severity"] == 2 for x in d), d)

    def test_symbols(self):
        names = [s["name"] for s in symbols(self.GOOD)]
        self.assertEqual(names[0], "node t")
        self.assertIn("output l", names)
        self.assertIn("on GO", names)

    def test_hover(self):
        # line 1 (0-based), on the word 'l'
        self.assertIn("output l: digital", hover(self.GOOD, 1, 10))
        self.assertIsNone(hover(self.GOOD, 0, 2))      # a keyword is not a declared name

    def test_format_edits(self):
        self.assertEqual(format_edits(self.GOOD), [])
        e = format_edits(MESSY)
        self.assertEqual(len(e), 1)
        self.assertEqual(e[0]["newText"], format_checked(MESSY))

    def test_imported_error_is_shown_on_line_1(self):
        d = tempfile.mkdtemp()
        open(os.path.join(d, "lib.rudra"), "w").write("const X = 1 +\n")
        main = 'import "lib.rudra"\nnode t target nano {\n  heartbeat 3s\n}\n'
        out = diagnostics(main, os.path.join(d, "main.rudra"))
        self.assertTrue(out)
        self.assertTrue(all(x["range"]["start"]["line"] == 0 for x in out), out)


def frame(obj):
    b = json.dumps(obj).encode()
    return b"Content-Length: " + str(len(b)).encode() + b"\r\n\r\n" + b


def parse_frames(data):
    out = []
    while data:
        head, _, rest = data.partition(b"\r\n\r\n")
        n = int(head.split(b":")[1])
        out.append(json.loads(rest[:n]))
        data = rest[n:]
    return out


class LspSessionTests(unittest.TestCase):
    def test_session_over_stdio(self):
        uri = "file:///tmp/x.rudra"
        good = LspFunctionTests.GOOD
        msgs = [
            {"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {}},
            {"jsonrpc": "2.0", "method": "initialized", "params": {}},
            {"jsonrpc": "2.0", "method": "textDocument/didOpen",
             "params": {"textDocument": {"uri": uri, "text": good.replace("l = true", "lamp = true")}}},
            {"jsonrpc": "2.0", "method": "textDocument/didChange",
             "params": {"textDocument": {"uri": uri}, "contentChanges": [{"text": good}]}},
            {"jsonrpc": "2.0", "id": 2, "method": "textDocument/documentSymbol", "params": {"textDocument": {"uri": uri}}},
            {"jsonrpc": "2.0", "id": 3, "method": "textDocument/hover",
             "params": {"textDocument": {"uri": uri}, "position": {"line": 1, "character": 10}}},
            {"jsonrpc": "2.0", "id": 4, "method": "nonsense", "params": {}},
            {"jsonrpc": "2.0", "id": 5, "method": "shutdown", "params": {}},
            {"jsonrpc": "2.0", "method": "exit"},
        ]
        env = dict(os.environ, PYTHONPATH=ROOT)
        r = subprocess.run([sys.executable, "-m", "rudrac", "lsp"], input=b"".join(frame(m) for m in msgs),
                           capture_output=True, env=env, timeout=30)
        self.assertEqual(r.returncode, 0, r.stderr)
        out = parse_frames(r.stdout)
        by_id = {m["id"]: m for m in out if "id" in m}
        self.assertEqual(by_id[1]["result"]["serverInfo"]["name"], "rudrac")
        pubs = [m for m in out if m.get("method") == "textDocument/publishDiagnostics"]
        self.assertEqual(len(pubs), 2)
        self.assertEqual([x["severity"] for x in pubs[0]["params"]["diagnostics"]], [1, 2])
        self.assertEqual(pubs[1]["params"]["diagnostics"], [])
        self.assertTrue(by_id[2]["result"])
        self.assertIn("output l", by_id[3]["result"]["contents"]["value"])
        self.assertEqual(by_id[4]["error"]["code"], -32601)
        self.assertIsNone(by_id[5]["result"])


if __name__ == "__main__":
    unittest.main()
