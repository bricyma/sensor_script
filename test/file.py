import os
files = [f for f in os.listdir(self.current_folder) if os.path.isfile(os.path.join(self.current_folder, f))]
