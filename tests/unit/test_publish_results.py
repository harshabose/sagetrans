"""scripts/publish_results.py: the Pages branch, the link check and the refusals."""

import hashlib
import importlib.util
import json
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
_spec = importlib.util.spec_from_file_location("publish_results",
                                               ROOT / "scripts" / "publish_results.py")
assert _spec is not None and _spec.loader is not None
pub = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(pub)

PAGE_HTML = (
    "<html><body><a href='#top'>top</a><a href='C_params.csv'>t</a>"
    "<a href='./run_record.json'>r</a><a href='report.html?x=1#files'>self</a>"
    "<a href='https://example.org/x'>out</a><img src='fig.svg'><a href='data:text/plain,hi'>d</a>"
    "</body></html>"
)


def sh(cwd: Path, *args: str) -> str:
    return subprocess.run(["git", *args], cwd=cwd, check=True, capture_output=True,
                          text=True).stdout.strip()


@pytest.fixture
def repo(tmp_path):
    r = tmp_path / "repo"
    r.mkdir()
    sh(r, "init", "-q", "-b", "master")
    sh(r, "config", "user.name", "t")
    sh(r, "config", "user.email", "t@example.org")
    (r / "f.txt").write_text("x", encoding="utf-8")
    sh(r, "add", ".")
    sh(r, "commit", "-q", "-m", "init")
    return r


def make_folder(base: Path, *, commit="abcdef1", dirty=False, label="DRAFT", html=PAGE_HTML,
                extra=()):
    f = base / "study"
    f.mkdir()
    outputs = {}
    for name in ("C_params.csv", "fig.svg", *extra):
        (f / name).write_text(f"data {name}", encoding="utf-8")
        outputs[name] = hashlib.sha256((f / name).read_bytes()).hexdigest()
    (f / "report.html").write_text(html, encoding="utf-8")
    (f / "tables.xlsx").write_bytes(b"xlsx")
    (f / "run_record.json").write_text(json.dumps({
        "outputs": outputs, "git_commit": commit, "git_dirty": dirty, "label": label,
        "mode": "draft", "date": "2026-10-29", "source_hash": "ab" * 32,
        "reasons": ["72 register entries are TBD: a, b"], "library_versions": {"python": "3.13"},
    }), encoding="utf-8")
    return f


def test_the_branch_is_one_parentless_commit_with_everything_the_page_links_to(tmp_path, repo):
    folder = make_folder(tmp_path)
    before = sh(repo, "rev-parse", "HEAD"), sh(repo, "status", "--short")
    info = pub.publish(folder, repo)
    assert info["branch"] == "results/draft-abcdef1" and "pushed" not in info
    files = set(sh(repo, "ls-tree", "-r", "--name-only", info["branch"]).splitlines())
    assert files == {"index.html", "report.html", "C_params.csv", "fig.svg", "run_record.json",
                     "tables.xlsx", ".nojekyll", "README.md"}
    assert sh(repo, "rev-list", "--parents", "-n", "1", info["branch"]).split() == [info["commit"]]
    # the two pages are the same bytes, so the sha256 inside the page stays true
    assert (sh(repo, "rev-parse", f"{info['branch']}:index.html")
            == sh(repo, "rev-parse", f"{info['branch']}:report.html"))
    readme = sh(repo, "show", f"{info['branch']}:README.md")
    assert "abcdef1" in readme and "This is a draft" in readme and "72 register entries" in readme
    assert (sh(repo, "rev-parse", "HEAD"), sh(repo, "status", "--short")) == before  # untouched
    assert sh(repo, "branch", "--show-current") == "master"


def test_a_dry_run_checks_everything_and_creates_nothing(tmp_path, repo):
    info = pub.publish(make_folder(tmp_path), repo, dry_run=True)
    assert info["branch"] == "results/draft-abcdef1" and info["commit"] == ""
    assert sh(repo, "branch", "--list", "results/*") == ""


def test_a_page_that_would_404_is_not_published(tmp_path, repo):
    html = PAGE_HTML.replace("</body>", "<a href='B1_front.csv'>missing</a></body>")
    with pytest.raises(pub.PublishError, match="would 404|would not be on the branch") as exc:
        pub.publish(make_folder(tmp_path, html=html), repo)
    assert "B1_front.csv" in str(exc.value)
    assert sh(repo, "branch", "--list", "results/*") == ""


def test_the_relative_links_leave_out_anchors_external_urls_and_queries():
    assert pub.relative_links(PAGE_HTML) == ["./run_record.json", "C_params.csv", "fig.svg",
                                             "report.html"]
    assert pub.missing_links(PAGE_HTML, ["C_params.csv", "fig.svg", "report.html",
                                         "run_record.json"]) == []


def test_a_dirty_run_is_refused_unless_allowed(tmp_path, repo):
    folder = make_folder(tmp_path, dirty=True)
    with pytest.raises(pub.PublishError, match="dirty tree"):
        pub.publish(folder, repo)
    info = pub.publish(folder, repo, allow_dirty=True)
    assert "the tree was dirty" in sh(repo, "show", f"{info['branch']}:README.md")


def test_a_file_edited_or_missing_since_the_run_is_refused(tmp_path, repo):
    folder = make_folder(tmp_path)
    (folder / "C_params.csv").write_text("edited", encoding="utf-8")
    with pytest.raises(pub.PublishError, match="C_params.csv: edited after the run"):
        pub.publish(folder, repo)
    (folder / "C_params.csv").unlink()
    with pytest.raises(pub.PublishError, match="C_params.csv: missing"):
        pub.publish(folder, repo)


def test_no_report_no_record_or_no_commit_is_refused(tmp_path, repo):
    folder = make_folder(tmp_path)
    (folder / "report.html").unlink()
    with pytest.raises(pub.PublishError, match="--view"):
        pub.publish(folder, repo)
    with pytest.raises(pub.PublishError, match="run_record.json not found"):
        pub.publish(tmp_path, repo)
    with pytest.raises(pub.PublishError, match="no usable git commit"):
        pub.branch_name({"git_commit": "unknown", "label": "DRAFT"})


def test_the_name_follows_the_label_and_the_runs_own_commit():
    assert pub.branch_name({"git_commit": "ae83086", "label": "DRAFT"}) == "results/draft-ae83086"
    release = pub.branch_name({"git_commit": "ae83086", "label": "RELEASE"})
    assert release == "results/release-ae83086"
    with pytest.raises(pub.PublishError, match="not DRAFT or RELEASE"):
        pub.branch_name({"git_commit": "ae83086", "label": "weird"})


def test_an_existing_branch_is_never_overwritten(tmp_path, repo):
    folder = make_folder(tmp_path)
    pub.publish(folder, repo)
    with pytest.raises(pub.PublishError, match="already exists in this clone"):
        pub.publish(folder, repo)


def test_nothing_is_pushed_without_push_and_a_remote_branch_is_not_overwritten(tmp_path, repo):
    remote = tmp_path / "remote.git"
    sh(tmp_path, "init", "-q", "--bare", str(remote))
    sh(repo, "remote", "add", "origin", str(remote))
    folder = make_folder(tmp_path)
    info = pub.publish(folder, repo)
    assert sh(remote, "branch", "--list") == ""  # created locally, not pushed
    sh(repo, "branch", "-D", info["branch"])
    info = pub.publish(folder, repo, push=True)
    assert info["pushed"] == "origin"
    assert info["branch"] in sh(remote, "branch", "--list")
    sh(repo, "branch", "-D", info["branch"])
    with pytest.raises(pub.PublishError, match="already exists on origin"):
        pub.publish(folder, repo, push=True)


def test_the_default_folder_is_output_dir_and_run_name_of_study_yaml(tmp_path):
    cfg = tmp_path / "configs"
    cfg.mkdir()
    (cfg / "study.yaml").write_text(
        "run_name: study            # output\noutput_dir: results        # ignored\n",
        encoding="utf-8")
    assert pub.default_folder(tmp_path, cfg) == tmp_path / "results" / "study"
    (cfg / "study.yaml").write_text("mode: draft\n", encoding="utf-8")
    assert pub.default_folder(tmp_path, cfg) == tmp_path / "results" / "study"  # the defaults


def test_the_command_line_reports_a_refusal_and_exits_nonzero(tmp_path, repo, capsys):
    folder = make_folder(tmp_path, dirty=True)
    assert pub.main([str(folder), "--repo", str(repo)]) == 1
    assert "dirty tree" in capsys.readouterr().err
    assert pub.main([str(folder), "--repo", str(repo), "--allow-dirty", "--dry-run"]) == 0
    assert "every link in the page resolves" in capsys.readouterr().out
