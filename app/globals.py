import os
from background.prototype_manager import PrototypeManager
from app.services.admin_status import AdminStatusManager

# Initialize shared resources
thread_manager = PrototypeManager()
archiver_manager = None 
admin_status = AdminStatusManager()

# Directory for gallery images
GALLERY_PATH = os.path.join(os.getcwd(), "archive")

def set_archiver_manager(new_manager):
    global archiver_manager
    archiver_manager = new_manager