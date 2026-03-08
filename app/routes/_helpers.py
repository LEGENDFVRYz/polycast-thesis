
import os
from app.models.admin import Admin
from app.models.gallery import Gallery
from app.models.session import Session as DBSession

# Import globals
import app.globals as g


# ---------------------------------------------------------------------
# Gallery Page Helpers
# ---------------------------------------------------------------------

def get_current_admin():
    """Helper to fetch the currently logged-in admin."""
    admin_name = str(g.admin_status.get_field('admin_name'))
    return Admin.query.filter_by(username=admin_name).first()


def get_eligible_galleries(admin, view_type='all'):
    """
    Helper to fetch and validate galleries based on the view type.
    Handles the database querying and the disk validation.
    """
    if not admin:
        return []

    # Base query: Get all galleries under the logged-in admin
    base_query = Gallery.query.filter_by(admin_id=admin.id)

    # Apply appropriate filters
    if view_type == 'favorites':
        base_query = base_query.filter_by(is_favorite=True).filter(Gallery.deleted_at.is_(None))
    elif view_type == 'trash':
        base_query = base_query.filter(Gallery.deleted_at.is_not(None))
    else:  # 'all'
        base_query = base_query.filter(Gallery.deleted_at.is_(None))

    # Fetch galleries (favorites first, then alphabetically by name)
    admin_galleries = base_query.order_by(Gallery.is_favorite.desc(), Gallery.name).all()

    eligible_folders = []
    user_filepath = os.path.join(g.GALLERY_PATH, str(admin.id))

    # VALIDATION: Check if gallery folder exists on disk
    for gllry in admin_galleries:
        folder_path = os.path.join(user_filepath, str(gllry.id))
        if os.path.isdir(folder_path):
            eligible_folders.append({
                'id': gllry.id, 
                'name': gllry.name, 
                'is_favorite': gllry.is_favorite
            })

    return eligible_folders