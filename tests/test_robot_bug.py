#!/usr/bin/env python3
"""Offline tests for the GitLab issue command."""

import os
from argparse import Namespace
import subprocess
import tempfile
import unittest
from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec, spec_from_loader
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
BUG = ROOT / "bin" / "bug"


class GitLabCommandTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        loader = SourceFileLoader("robot_bug", str(BUG))
        spec = spec_from_loader(loader.name, loader)
        cls.bug = module_from_spec(spec)
        loader.exec_module(cls.bug)

    def test_env_loads_only_gitlab_credentials_and_keeps_exported_values(self):
        with tempfile.TemporaryDirectory() as temp:
            env_file = Path(temp) / ".env"
            env_file.write_text(
                "GITLAB_SITE=git.example.test\n"
                "GITLAB_USER=someone\n"
                "GITLAB_PASSWORD='two words # included'\n"
                "DATABASE_URL=not-for-this-command\n"
            )
            environ = {"GITLAB_USER": "override"}
            self.bug.load_gitlab_env(env_file, environ)
            self.assertEqual(environ["GITLAB_USER"], "override")
            self.assertEqual(environ["GITLAB_PASSWORD"], "two words # included")
            self.assertNotIn("DATABASE_URL", environ)

    def test_project_defaults_to_robot_not_rugent(self):
        with patch.dict(os.environ, {}, clear=True):
            self.assertEqual(self.bug.project_id(), "40")

    def test_branch_name_preserves_type_and_ascii_slug(self):
        self.assertEqual(self.bug.branch_name(["bug"], 12, "Fix wheels!"), "bug/12-fix-wheels")
        self.assertEqual(self.bug.branch_name(["type::feature"], 13, "Камера"), "feat/13")

    def test_new_issue_labels_do_not_copy_rugent_statuses(self):
        self.assertEqual(self.bug.new_labels(["sev::low"]), ["sev::low"])

    def test_default_branch_comes_from_gitlab(self):
        with patch.object(self.bug, "call", return_value={"default_branch": "feat/current"}):
            self.assertEqual(self.bug.default_branch(), "feat/current")
            self.bug.call.assert_called_once_with("GET", "")

    def test_api_call_targets_robot_project(self):
        class Response:
            def __enter__(self):
                return self

            def __exit__(self, *_):
                pass

            def read(self):
                return b'{"path_with_namespace":"ermak/robot"}'

        with patch.dict(os.environ, {"GITLAB_SITE": "git.example.test", "GITLAB_PROJECT_ID": "40"}):
            with patch.object(self.bug, "token", return_value="test-token"):
                with patch.object(self.bug.urllib.request, "urlopen", return_value=Response()) as open_url:
                    self.assertEqual(self.bug.call("GET", "")["path_with_namespace"], "ermak/robot")
                    request = open_url.call_args.args[0]
                    self.assertEqual(request.full_url, "https://git.example.test/api/v4/projects/40")
                    self.assertEqual(request.get_header("Authorization"), "Bearer test-token")

    def test_mr_title_does_not_close_issue_implicitly(self):
        self.assertEqual(self.bug.strip_closing("fix: wheel (Closes #12)"), "fix: wheel (#12)")

    def test_epic_new_creates_an_organizing_issue(self):
        args = Namespace(title="Rust-эффектор", body_file=None, label=[])
        with patch.object(self.bug, "body_from", return_value="Цель и критерии"):
            with patch.object(self.bug, "call", return_value={"iid": 5, "web_url": "https://example.test/5"}) as call:
                with patch("builtins.print"):
                    self.bug.command_epic_new(args)
        call.assert_called_once_with("POST", "/issues", {
            "title": "[Эпик] Rust-эффектор",
            "description": "Цель и критерии",
            "labels": "",
        })

    def test_new_task_links_back_and_updates_epic_checklist(self):
        args = Namespace(title="Датчики", body_file=None, label=[], epic=5)
        epic = {"title": "[Эпик] Rust-эффектор", "description": "Цель и критерии"}
        issue = {"iid": 6, "web_url": "https://example.test/6"}
        with patch.object(self.bug, "body_from", return_value="Измерить частоту"):
            with patch.object(self.bug, "call", side_effect=[epic, issue, issue]) as call:
                with patch("builtins.print"):
                    self.bug.command_new(args)
        self.assertEqual(call.call_args_list[0].args, ("GET", "/issues/5"))
        self.assertEqual(call.call_args_list[1].args, ("POST", "/issues", {
            "title": "Датчики",
            "description": "Эпик: #5\n\nИзмерить частоту",
            "labels": "",
        }))
        self.assertEqual(call.call_args_list[2].args, ("PUT", "/issues/5", {
            "description": "Цель и критерии\n\n## Задачи\n\n- [ ] #6 Датчики",
        }))

    def test_new_task_appends_to_existing_epic_checklist(self):
        args = Namespace(title="Командер", body_file=None, label=[], epic=5)
        epic = {"title": "[Эпик] Rust-эффектор", "description": "Цель\n\n## Задачи\n\n- [ ] #6 Датчики"}
        issue = {"iid": 7, "web_url": "https://example.test/7"}
        with patch.object(self.bug, "body_from", return_value="TUI"):
            with patch.object(self.bug, "call", side_effect=[epic, issue, issue]) as call:
                with patch("builtins.print"):
                    self.bug.command_new(args)
        self.assertEqual(call.call_args_list[2].args[2]["description"],
                         "Цель\n\n## Задачи\n\n- [ ] #6 Датчики\n- [ ] #7 Командер")

    def test_new_task_rejects_a_non_epic_parent_before_creating(self):
        args = Namespace(title="Датчики", body_file=None, label=[], epic=5)
        with patch.object(self.bug, "body_from", return_value="Описание"):
            with patch.object(self.bug, "call", return_value={"title": "Обычная задача", "description": ""}) as call:
                with self.assertRaises(SystemExit):
                    self.bug.command_new(args)
        call.assert_called_once_with("GET", "/issues/5")

    def test_robot_bug_dispatches_without_contacting_robot(self):
        with tempfile.TemporaryDirectory() as temp:
            fake_bug = Path(temp) / "bug"
            fake_bug.write_text("#!/bin/sh\nprintf '%s\\n' \"$*\"\n")
            fake_bug.chmod(0o700)
            result = subprocess.run(
                [str(ROOT / "bin" / "robot"), "bug", "list", "--all"],
                capture_output=True,
                text=True,
                env={**os.environ, "ROBOT_BUG_BIN": str(fake_bug), "ROBOT_SSH": "/does/not/exist"},
                check=True,
            )
            self.assertEqual(result.stdout.strip(), "list --all")


if __name__ == "__main__":
    unittest.main()
