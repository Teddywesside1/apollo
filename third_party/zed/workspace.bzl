""" Loads the ZED library"""


def clean_dep(dep):
    return str(Label(dep))
 
def repo():
    native.new_local_repository(
        name = "zed",
        build_file = clean_dep("//third_party/zed:zed.BUILD"),
        path = "/usr/local/zed",
    )