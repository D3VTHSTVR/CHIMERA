"""

-----------------------------------------------------------------------
Copyright 2018-2020 Jonathan Arreguit, Shravan Tata Ramalingasetty
Copyright 2018 BioRobotics Laboratory, École polytechnique fédérale de Lausanne

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
-----------------------------------------------------------------------

Utility classes and methods for farms_sdf

"""

import os
from dataclasses import dataclass

from treelib import Tree


class InvalidJointError(ValueError):
    """Raised when a joint name is not found in the model."""
    pass


class InvalidLinkError(ValueError):
    """Raised when a link name is not found in the model."""
    pass


class EmptyModelError(ValueError):
    """Raised when the model has no root link (e.g. no links or invalid structure)."""
    pass


def replace_file_name_in_path(file_path, new_name):
    """
    Replace a file name in a given path. File extension is retained

    Parameters
    ----------
    file_path : <str>
        Path to the file object
    new_name : <str>
        Name to replace the original file name
    Returns
    -------
    out : <str>
        New path with the replaced file name
    """
    full_path = os.path.split(file_path)[0]
    file_extension = (os.path.split(file_path)[-1]).split('.')[-1]
    new_name = new_name + '.' + file_extension
    return os.path.join(full_path, new_name)


def link_name_to_index(model):
    """ Generate a dictionary for link names and their indicies in the
    model. """
    return {
        link.name : index for index, link in enumerate(model.links)
    }


def joint_name_to_index(model):
    """ Generate a dictionary for joint names and their indices in the model. """
    return {
        joint.name: index for index, joint in enumerate(model.joints)
    }


def find_parent_joints(model, joint_name):
    """ Find all the joints parented to the given joint. """
    joint_id = joint_name_to_index(model)
    if joint_name not in joint_id:
        raise InvalidJointError(
            "Joint '{}' not found in model. "
            "Valid joints: {}.".format(
                joint_name,
                ", ".join(sorted(joint_id.keys()))
            )
        )
    joint = model.joints[joint_id[joint_name]]
    plink = joint.parent
    return [
        j.name for j in model.joints if j.child == plink
    ]


def find_child_joints(model, joint_name):
    """ Find all the joints that are children of the given joint. """
    joint_id = joint_name_to_index(model)
    if joint_name not in joint_id:
        raise InvalidJointError(
            "Joint '{}' not found in model. "
            "Valid joints: {}.".format(
                joint_name,
                ", ".join(sorted(joint_id.keys()))
            )
        )
    joint = model.joints[joint_id[joint_name]]
    clink = joint.child
    return [
        j.name for j in model.joints if j.parent == clink
    ]


def find_neighboring_joints(model, joint):
    """Find parent, child, and sibling neighboring joints.

    Includes: joints that share the same parent link (siblings), the parent
    joint(s) of this joint, and the child joint(s) of this joint.
    """
    joint_id = joint_name_to_index(model)
    if joint not in joint_id:
        raise InvalidJointError(
            "Joint '{}' not found in model. "
            "Valid joints: {}.".format(
                joint,
                ", ".join(sorted(joint_id.keys()))
            )
        )
    j_obj = model.joints[joint_id[joint]]
    parent_link = j_obj.parent
    child_link = j_obj.child
    # Parent joints (joints whose child link is our parent link)
    parent_joints = [jo.name for jo in model.joints if jo.child == parent_link]
    # Child joints (joints whose parent link is our child link)
    child_joints = [jo.name for jo in model.joints if jo.parent == child_link]
    # Sibling joints (joints that share the same parent link, excluding self)
    sibling_joints = [
        jo.name for jo in model.joints
        if jo.parent == parent_link and jo.name != joint
    ]
    return list(dict.fromkeys(parent_joints + sibling_joints + child_joints))


def find_link_joints(model, link_name):
    """Find the joints attached to a given link (joints whose parent is link_name).

    Parameters
    ----------
    model : <ModelSDF>
        SDF model

    link_name : <str>
        Name of the link in the sdf

    Returns
    -------
    out : <tuple>
        Tuple of joint names attached to the link

    Raises
    ------
    InvalidLinkError
        If link_name is not found in the model.
    """
    link_id = link_name_to_index(model)
    if link_name not in link_id:
        raise InvalidLinkError(
            "Link '{}' not found in model. "
            "Valid links: {}.".format(
                link_name,
                ", ".join(sorted(link_id.keys()))
            )
        )
    return tuple([
        joint.name
        for joint in model.joints
        if joint.parent == link_name
    ])


def find_root(model):
    """Find the root link (the link that is never a child of any joint).

    Parameters
    ----------
    model : <ModelSDF>
        SDF model with links and joints

    Returns
    -------
    root_name : str
        Name of the root link

    Raises
    ------
    EmptyModelError
        If the model has no links or no root link could be determined.
    """
    if not model.links:
        raise EmptyModelError("Model has no links.")
    child_links = {j.child for j in model.joints}
    for link in model.links:
        if link.name not in child_links:
            return link.name
    raise EmptyModelError(
        "No root link found: every link is a child of some joint. "
        "Model structure may be invalid (e.g. cycles)."
    )


@dataclass
class TreeData:
    """Data for SDF tree
    """
    joint: str


def add_nodes_to_tree(model, tree, links=None, joint_index=None):
    """ Add nodes to links """
    if joint_index is None:
        joint_index = joint_name_to_index(model)
    if len(links) == 0:
        return True
    #: New links to be added
    new_links = []
    for link in links:
        for joint_name in find_link_joints(model, link):
            joint = model.joints[joint_index[joint_name]]
            tree.create_node(
                tag=joint.child, identifier=joint.child,
                parent=joint.parent, data=TreeData(joint.name)
            )
            new_links.append(joint.child)
    add_nodes_to_tree(model, tree, new_links, joint_index)


def construct_tree(model) -> Tree:
    """ Construct tree. """
    tree = Tree()
    root = find_root(model)
    tree.create_node(tag=root, identifier=root, parent=None, data=TreeData(""))
    joint_index = joint_name_to_index(model)
    add_nodes_to_tree(model, tree, links=[root], joint_index=joint_index)
    return tree


def get_all_subtrees(model):
    """ Get all the subtrees in the given model. """
    #: Construct the tree
    tree = construct_tree(model)
    #: Get the branch nodes
    branch_roots = [
        n.identifier
        for n in tree.all_nodes_itr()
        if len(tree.children(n.identifier)) > 1
    ]
    #: Get all subtrees
    return [
        tree.remove_subtree(children.identifier)
        for root in branch_roots[::-1]
        for children in tree.children(root)
    ]
