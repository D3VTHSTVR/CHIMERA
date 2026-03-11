"""Unit tests for NeuroMechFly.sdf.utils."""
import pytest

from NeuroMechFly.sdf.utils import (
    replace_file_name_in_path,
    link_name_to_index,
    joint_name_to_index,
    find_parent_joints,
    find_child_joints,
    find_neighboring_joints,
    find_link_joints,
    find_root,
    construct_tree,
    get_all_subtrees,
    InvalidJointError,
    InvalidLinkError,
    EmptyModelError,
)


class TestReplaceFileNameInPath:
    def test_basic(self):
        assert replace_file_name_in_path("/a/b/file.txt", "new") == "/a/b/new.txt"

    def test_preserves_extension(self):
        assert replace_file_name_in_path("path/model.sdf", "animat") == "path/animat.sdf"


class TestLinkNameToIndex:
    def test_simple_model(self, simple_model):
        idx = link_name_to_index(simple_model)
        assert idx["root"] == 0
        assert idx["link1"] == 1
        assert idx["link2"] == 2


class TestJointNameToIndex:
    def test_simple_model(self, simple_model):
        idx = joint_name_to_index(simple_model)
        assert idx["joint_root_1"] == 0
        assert idx["joint_1_2"] == 1


class TestFindParentJoints:
    def test_valid_joint(self, simple_model):
        assert find_parent_joints(simple_model, "joint_root_1") == []
        assert find_parent_joints(simple_model, "joint_1_2") == ["joint_root_1"]

    def test_invalid_joint_raises(self, simple_model):
        with pytest.raises(InvalidJointError) as exc_info:
            find_parent_joints(simple_model, "nonexistent")
        assert "nonexistent" in str(exc_info.value)


class TestFindChildJoints:
    def test_valid_joint(self, simple_model):
        assert find_child_joints(simple_model, "joint_root_1") == ["joint_1_2"]
        assert find_child_joints(simple_model, "joint_1_2") == []

    def test_invalid_joint_raises(self, simple_model):
        with pytest.raises(InvalidJointError) as exc_info:
            find_child_joints(simple_model, "nonexistent")
        assert "nonexistent" in str(exc_info.value)


class TestFindNeighboringJoints:
    def test_chain(self, simple_model):
        assert set(find_neighboring_joints(simple_model, "joint_root_1")) == {"joint_1_2"}
        assert set(find_neighboring_joints(simple_model, "joint_1_2")) == {"joint_root_1"}

    def test_branch(self, branched_model):
        neigh = find_neighboring_joints(branched_model, "j1")
        assert set(neigh) == {"j2"}


class TestFindLinkJoints:
    def test_valid_link(self, simple_model):
        assert find_link_joints(simple_model, "root") == ("joint_root_1",)
        assert find_link_joints(simple_model, "link1") == ("joint_1_2",)

    def test_invalid_link_raises(self, simple_model):
        with pytest.raises(InvalidLinkError) as exc_info:
            find_link_joints(simple_model, "nonexistent")
        assert "nonexistent" in str(exc_info.value)


class TestFindRoot:
    def test_simple_model(self, simple_model):
        assert find_root(simple_model) == "root"

    def test_branched_model(self, branched_model):
        assert find_root(branched_model) == "root"

    def test_empty_model_raises(self, empty_model):
        with pytest.raises(EmptyModelError):
            find_root(empty_model)


class TestConstructTree:
    def test_simple_model(self, simple_model):
        tree = construct_tree(simple_model)
        assert tree.root == "root"
        # Tree structure: root -> link1 -> link2
        children_root = [n.identifier for n in tree.children("root")]
        assert "link1" in children_root
        children_link1 = [n.identifier for n in tree.children("link1")]
        assert "link2" in children_link1


class TestGetAllSubtrees:
    def test_branched_model(self, branched_model):
        subtrees = get_all_subtrees(branched_model)
        # Two branches from root
        assert len(subtrees) == 2
