"""PyLabware utility functions for reply parsing"""

import re
from typing import List, Union


def slicer(reply: str, *args) -> Union[List, str]:
    """This is a wrapper function for reply parsing to provide consistent
    arguments order.

    Args:
        reply: Sequence object to slice.

    Returns:
        (any): Slice of the original object.
    """

    #FIXME
    # Make behavior consistent with list[index] access
    reply = reply[slice(*args)]
    if (len(reply) == 1):
        reply = str(reply[0])
    return reply


def researcher(reply, *args):
    """This is a wrapper function for reply parsing to provide consistent
    arguments order.

    Args:
        reply: Reply to parse with regular expression.

    Returns:
        (re.Match): Regular expression match object.
    """

    return re.search(*args, reply)


def stripper(reply: str, prefix=None, suffix=None) -> str:
    """This is a helper function used to strip off reply prefix and
    terminator. Standard Python str.strip() doesn't work reliably because
    it operates on character-by-character basis, while prefix/terminator
    is usually a group of characters.

    Args:
        reply: String to be stripped.
        prefix: Substring to remove from the beginning of the line.
        suffix: Substring to remove from the end of the line.

    Returns:
        (str): Naked reply.
    """

    if prefix is not None and reply.startswith(prefix):
        reply = reply[len(prefix):]

    if suffix is not None and reply.endswith(suffix):
        reply = reply[:-len(suffix)]

    return reply

def splitter(reply: str, separator: str, *slice_positions):
    """ This is a combination of str.split() followed by slicing.
        Allows to avoid using RegExes for simple cases.

    Args:
        reply (str): String to process
        separator (str): Separator for str.split().
    """

    reply = reply.split(separator)
    return slicer(reply, *slice_positions)