"""Pytest fixtures for NeuroMechFly tests."""
import pytest


def _make_mock_link(name):
    """Create a minimal mock link with a name attribute."""
    return type("Link", (), {"name": name})()


def _make_mock_joint(name, parent_link_name, child_link_name):
    """Create a minimal mock joint with name, parent, child (link names)."""
    return type("Joint", (), {
        "name": name,
        "parent": parent_link_name,
        "child": child_link_name,
    })()


@pytest.fixture
def simple_model():
    """Minimal SDF-like model: root -> link1 -> link2 (chain)."""
    links = [_make_mock_link(n) for n in ("root", "link1", "link2")]
    joints = [
        _make_mock_joint("joint_root_1", "root", "link1"),
        _make_mock_joint("joint_1_2", "link1", "link2"),
    ]
    model = type("Model", (), {"links": links, "joints": joints})()
    return model


@pytest.fixture
def branched_model():
    """Model with branching: root -> link1, root -> link2."""
    links = [_make_mock_link(n) for n in ("root", "link1", "link2")]
    joints = [
        _make_mock_joint("j1", "root", "link1"),
        _make_mock_joint("j2", "root", "link2"),
    ]
    model = type("Model", (), {"links": links, "joints": joints})()
    return model


@pytest.fixture
def empty_model():
    """Model with no links (truly empty, find_root should raise)."""
    links = []
    joints = []
    model = type("Model", (), {"links": links, "joints": joints})()
    return model
