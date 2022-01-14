import os
import shutil

class PathCreator:
    def __init__(self, username, old_path, theme_name):
        self.username = username
        self.old_path = old_path
        self.theme_name = theme_name
        self.dir_ = os.path.join("C:\\Users\\", username, "Shuffleboard\\themes")
        self.new_path = os.path.join(self.dir_, self.theme_name)
        
    def make_themepath(self):
        if not os.path.exists(self.new_path):
            shutil.copyfile(self.old_path, self.new_path)

P = PathCreator("david", "testfile.txt", "testfile.txt")
P.make_themepath()