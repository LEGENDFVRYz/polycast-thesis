import os
from datetime import datetime

from flask import Blueprint, render_template, redirect, url_for, request, session, jsonify, flash, send_from_directory, abort
from app import db
from app.models.admin import Admin
from app.models.gallery import Gallery
from app.models.session import Session as DBSession
from app.models.note import Note
from app.routes._helpers import get_current_admin, get_eligible_galleries
from config import BROWSER_WS_PORT

# Import globals
import app.globals as g

gallery_bp = Blueprint('gallery', __name__)


def _get_session_counts(folders):
    """
    Returns {gallery_id: active_session_count} in one aggregation query.
    Works with folders as either ORM objects or dicts.
    """
    if not folders:
        return {}
 
    # Support both ORM objects and plain dicts
    def _id(f):
        return f['id'] if isinstance(f, dict) else f.id
 
    gallery_ids = [_id(f) for f in folders]
 
    rows = (
        db.session.query(
            DBSession.gallery_id,
            db.func.count(DBSession.id).label('count')
        )
        .filter(
            DBSession.gallery_id.in_(gallery_ids),
            DBSession.deleted_at.is_(None)
        )
        .group_by(DBSession.gallery_id)
        .all()
    )
 
    counts = {row.gallery_id: row.count for row in rows}
 
    # Default to 0 for any gallery with no sessions yet
    for f in folders:
        counts.setdefault(_id(f), 0)
 
    return counts


def _get_recent_sessions(limit=4):
    """
    Returns the most recently created or updated sessions across all galleries.
    Returns a list of sessions ordered by last activity or creation date.
    """
    sessions = (
        db.session.query(DBSession)
        .filter(DBSession.deleted_at.is_(None))
        .order_by(DBSession.last_activity_at.desc().nulls_last(), DBSession.created_at.desc())
        .limit(limit)
        .all()
    )
    return sessions


# ---------------------------------------------------------------------
# General Gallery Page Routes
# - Waiting page for the clients if the stream is not yet setup
# ---------------------------------------------------------------------
@gallery_bp.route("/gallery")
def index():
    """
    Dashboard/Home View.
    Shows shortcuts (recents, trash) alongside the galleries.
    """
    admin = get_current_admin()
    folders = get_eligible_galleries(admin, view_type='all')
    is_setup = g.admin_status.get_field("hosting_active") if admin else False
    
    is_current_user = ('user' in session) and (session['user'] == str(g.admin_status.get_field('admin_name')))
    recent_sessions = _get_recent_sessions(limit=4)
    
    return render_template(
        "gallery/gallery.html",
        is_setup=is_setup,
        folders=folders,
        session_counts=_get_session_counts(folders),
        is_current_user=is_current_user,
        recent_sessions=recent_sessions,
    )


@gallery_bp.route("/gallery/all")
def all_galleries():
    """
    Dedicated All Galleries View.
    Strictly focuses on displaying the full list of gallery folders.
    """
    admin = get_current_admin()
    folders = get_eligible_galleries(admin, view_type='all')
    is_setup = g.admin_status.get_field("hosting_active") if admin else False
    
    is_current_user = ('user' in session) and (session['user'] == str(g.admin_status.get_field('admin_name')))
    
    return render_template(
        "gallery/gallery-all.html", 
        is_setup=is_setup, 
        session_counts=_get_session_counts(folders),
        is_current_user=is_current_user,
        folders=folders
    )


@gallery_bp.route("/gallery/favorites")
def favorites():
    """List only favorite galleries."""
    admin = get_current_admin()
    folders = get_eligible_galleries(admin, view_type='favorites')
    is_setup = g.admin_status.get_field("hosting_active") if admin else False
    
    is_current_user = ('user' in session) and (session['user'] == str(g.admin_status.get_field('admin_name')))
    
    return render_template(
        "gallery/gallery-fav.html", 
        is_setup=is_setup, 
        session_counts=_get_session_counts(folders),
        is_current_user=is_current_user,
        folders=folders
    )


@gallery_bp.route("/gallery/trash")
def trash():
    """List deleted/trashed galleries."""
    admin = get_current_admin()
    folders = get_eligible_galleries(admin, view_type='trash')
    is_setup = g.admin_status.get_field("hosting_active") if admin else False
    
    is_current_user = ('user' in session) and (session['user'] == str(g.admin_status.get_field('admin_name')))
    
    return render_template(
        "gallery/gallery-bin.html", 
        is_setup=is_setup, 
        session_counts=_get_session_counts(folders),
        is_current_user=is_current_user,
        folders=folders
    )



@gallery_bp.route("/gallery/<string:galleryname>")
def session_page(galleryname):
    """
    List all sessions within the selected gallery
    """

    current_view = request.args.get('view', 'all')
    
    # EDGE CASES: Protecting folders which not under by the logged in admin 
    admin_name = str(g.admin_status.get_field('admin_name'))
    current_admin = Admin.query.filter_by(username=admin_name).first()
    
    if not current_admin:
        return "Admin not found", 404
    
    # CHECK: If user is the currently logged in admin
    is_current_user = ('user' in session) and (session['user'] == admin_name)
    

    # QUERY: Fetch requested gallery (ANY state first)
    gallery_item = Gallery.query.filter_by(name=galleryname, admin_id=current_admin.id).first()

    if not gallery_item:
        return "Gallery not found", 404
    
    # CHECK: If gallery is deleted, show modal instead of error
    is_gallery_deleted = gallery_item.deleted_at is not None
    
    if is_gallery_deleted:
        # Pass minimal data and show recovery modal
        return render_template(
            "gallery/session.html",
            selected_gallery=gallery_item.name,
            gallery=gallery_item,
            sessions=[],
            session_stats={},
            active_view='all',
            is_current_user=is_current_user,
            is_gallery_deleted=True,
            now=datetime.utcnow()
        )

    # Path to the specific gallery folder
    gallery_filepath = os.path.join(g.GALLERY_PATH, str(current_admin.id), str(gallery_item.id))
    
    # QUERY: Fetch all the session under the validated gallery, apply all filters
    base_query = DBSession.query.filter_by(gallery_id=gallery_item.id)
    
    if current_view == 'trash':
        sessions_db = base_query.filter(DBSession.deleted_at.is_not(None)).all()
    else:
        current_view = 'all'
        sessions_db = base_query.filter(DBSession.deleted_at.is_(None)).all()

    # VALIDATION: Include sessions whose folders exist on disk
    sessions = []
    for s in sessions_db:
        folder_path = os.path.join(gallery_filepath, str(s.id))
        if os.path.isdir(folder_path):
            sessions.append(s)

    # STATS: Compute note count and total file size per session
    session_ids = [s.id for s in sessions]
    stats_rows = (
        db.session.query(
            Note.session_id,
            db.func.count(Note.id).label('note_count'),
            db.func.coalesce(db.func.sum(Note.file_size_bytes), 0).label('total_bytes')
        )
        .filter(Note.session_id.in_(session_ids))
        .group_by(Note.session_id)
        .all()
    ) if session_ids else []

    def _fmt_size(num_bytes):
        """Human-readable file size. Kept short for card display."""
        for unit in ('B', 'KB', 'MB', 'GB'):
            if num_bytes < 1024:
                return f"{num_bytes:.1f} {unit}" if unit != 'B' else f"{int(num_bytes)} B"
            num_bytes /= 1024
        return f"{num_bytes:.1f} TB"

    session_stats = {
        row.session_id: {
            'note_count': row.note_count,
            'file_size':  _fmt_size(row.total_bytes),
        }
        for row in stats_rows
    }
    # Ensure every session has a default entry even if it has zero notes yet
    for s in sessions:
        session_stats.setdefault(s.id, {'note_count': 0, 'file_size': '0 B'})
    
    
    return render_template(
        "gallery/session.html",
        selected_gallery=gallery_item.name,
        gallery=gallery_item,
        sessions=sessions,
        session_stats=session_stats,
        active_view=current_view,
        is_current_user=is_current_user,
        now=datetime.utcnow()
    )



@gallery_bp.route("/gallery/<string:galleryname>/<string:sessionname>")
def view_page(galleryname, sessionname):
    """
    Show all images/notes within the selected folder (or session)
    """
    image_extensions = {'.jpg', '.png'}
    
    # EDGE CASES: Protecting folders which not under by the logged in admin 
    admin_name = str(g.admin_status.get_field('admin_name'))
    current_admin = Admin.query.filter_by(username=admin_name).first()
    
    if not current_admin:
        return "Admin not found", 404
    

    # VALIDATION: Fetch the specific gallery of the selected session, 
    # ensuring it is belong to logged in admin and excluding soft-deleted
    gallery_item  =  Gallery.query.filter_by(name=galleryname, admin_id=current_admin.id) \
                            .filter(Gallery.deleted_at.is_(None)) \
                            .first()
    if not gallery_item:
        return "Gallery not found or deleted", 404

    # VALIDATION: Fetch the specific session reference, excluding soft-deleted
    session_obj =  DBSession.query.filter_by(name=sessionname, gallery_id=gallery_item.id) \
                            .filter(DBSession.deleted_at.is_(None)) \
                            .first()
    if not session_obj:
        return "Session not found or deleted", 404
    

    # Path to the session folder using ids
    folder_path = os.path.join(g.GALLERY_PATH, str(current_admin.id), str(gallery_item.id), str(session_obj.id))

    if not os.path.isdir(folder_path):
        print("FOLDER: Does not exist")
        return redirect(url_for("gallery.index"))

    try:
        with os.scandir(folder_path) as folder:
            images = sorted(
                (item.name for item in folder if item.is_file() and os.path.splitext(item.name)[1].lower() in image_extensions),
                key=str.lower
            )
    except (FileNotFoundError, PermissionError):
        return redirect(url_for("gallery.index"))
    
    # This url will be used for API processing of the rendering notes/images
    base_url = f"{current_admin.id}/{gallery_item.id}/{session_obj.id}"
    
    return render_template(
        "gallery/view.html",
        admin_name=admin_name,
        galleryname=gallery_item.name,
        sessionname=session_obj.name,
        images=images,
        base_url=base_url
    )


@gallery_bp.route("/gallery_images/<path:filename>")
def serve_image(filename):
    """
    Serve dynamically generated images from archive.
    filename: relative path inside <admin_id>/<gallery>/<session>/<image>
    """
    # Full filesystem path
    requested_path = os.path.abspath(os.path.join(g.GALLERY_PATH, filename))
    safe_base = os.path.abspath(g.GALLERY_PATH) + os.sep

    # CHECK: prevent directory traversal
    if not requested_path.startswith(safe_base):
        print(f"Forbidden attempt: {requested_path}")
        abort(403)

    if not os.path.isfile(requested_path):
        print(f"File not found: {requested_path}")
        abort(404)
    
    relative_path = os.path.relpath(requested_path, g.GALLERY_PATH).replace(os.sep, "/")
    return send_from_directory(g.GALLERY_PATH, relative_path)


@gallery_bp.route("/api/gallery/<string:gallery_name>/sessions")
def get_gallery_sessions(gallery_name):
    """
    Return all session names for a given gallery (by name) as JSON.
    - Used for dynamic dropdowns for user suggestions 
    """

    admin_name = str(g.admin_status.get_field('admin_name'))
    current_admin = Admin.query.filter_by(username=admin_name).first()

    if not current_admin:
        return jsonify({"sessions": []})

    gallery_item = Gallery.query.filter_by(admin_id=current_admin.id, name=gallery_name).first()
    if not gallery_item:
        return jsonify({"sessions": []})

    sessions = DBSession.query.filter_by(gallery_id=gallery_item.id).order_by(DBSession.created_at.desc()).all()
    session_names = [s.name for s in sessions]

    return jsonify({"sessions": session_names})



# ---------------------------------------------------------------------
# Gallery CRUD Operations Routes
# - CRUD + toggle favorite status + edit descriptions
# ---------------------------------------------------------------------
@gallery_bp.route("/gallery/<int:gallery_id>/delete", methods=['POST'])
def delete_gallery(gallery_id):

    # VALIDATION: Protect the operation to non-authorize person
    if "user" not in session:
        flash("You must be admin in to perform this action.", "error")
        return redirect(url_for("admin.login_page"))
    
    # CHECK: Find the specific gallery or return a 404 error
    gallery_item = Gallery.query.get_or_404(gallery_id)
    
    # VALIDATION: Ensure the gallery belongs to the logged in admin
    if gallery_item.admin_id != session['id'] : abort(403)
    
    
    try:
        gallery_item.delete(commit=False)   # custom softdelete method
        db.session.commit()
        
        flash('Gallery has been moved to the recycle bin.', 'success')
    except Exception as e:
        db.session.rollback()
        flash(f'Error: {e}', 'danger')
    
    return redirect(request.referrer or url_for('gallery.index'))

@gallery_bp.route("/gallery/<int:gallery_id>/update", methods=['POST'])
def update_gallery_name(gallery_id):

    # VALIDATION: Protect the operation to non-authorize person
    if "user" not in session:
        flash("You must be admin in to perform this action.", "error")
        return redirect(url_for("admin.login_page"))
    
    # CHECK: Find the specific gallery or return a 404 error
    gallery_item = Gallery.query.get_or_404(gallery_id)
    
    # VALIDATION: Ensure the gallery belongs to the logged in admin
    if gallery_item.admin_id != session['id'] : abort(403)
    
    # FETCH: Get the new name from the form
    new_name = request.form.get('gallery_name', '').strip()
    if not new_name:
        flash('A gallery name is required.', 'error')
        return redirect(url_for('admin.index'))

    try:
        gallery_item.name = new_name
        db.session.commit()
        
        flash('Gallery name updated successfully.', 'success')
    except Exception as e:
        db.session.rollback()
        flash(f'Error: {e}', 'danger')
    
    return redirect(request.referrer or url_for('gallery.index'))

@gallery_bp.route("/gallery/<int:gallery_id>/recover", methods=['POST'])
def recover_gallery(gallery_id):
    
    # VALIDATION: Protect the operation to non-authorize person
    if "user" not in session:
        flash("You must be admin in to perform this action.", "error")
        return redirect(url_for("admin.login_page"))
    
    # CHECK: Find the specific gallery or return a 404 error
    gallery_item = Gallery.query.get_or_404(gallery_id)
    
    # VALIDATION: Ensure the gallery belongs to the logged in admin
    if gallery_item.admin_id != session['id'] : abort(403)
    
    
    try:
        if hasattr(gallery_item, 'restore'):
            gallery_item.restore(commit=False)      # Softdelete Methods
        else:
            gallery_item.is_deleted = False         # Manual, incase of error
        
        db.session.commit()
        flash('Gallery has been successfully recovered from the bin.', 'success')

    except Exception as e:
        db.session.rollback()
        flash(f'Error: {e}', 'danger')
    
    return redirect(request.referrer or url_for('gallery.index'))

@gallery_bp.route("/gallery/<int:gallery_id>/force-delete", methods=['POST'])
def force_delete_gallery(gallery_id):
    
    # VALIDATION: Protect the operation to non-authorize person
    if "user" not in session:
        flash("You must be admin in to perform this action.", "error")
        return redirect(url_for("admin.login_page"))
    
    # CHECK: Find the specific gallery or return a 404 error
    gallery_item = Gallery.query.get_or_404(gallery_id)
    
    # VALIDATION: Ensure the gallery belongs to the logged in admin
    if gallery_item.admin_id != session['id'] : abort(403)
    
    
    try:
        db.session.delete(gallery_item)
        db.session.commit()
        flash('Gallery has been permanently deleted.', 'success')
    except Exception as e:
        db.session.rollback()
        flash(f'Error: {e}', 'danger')
    
    return redirect(request.referrer or url_for('gallery.index'))


@gallery_bp.route("/gallery/<int:gallery_id>/favorite", methods=['POST'])
def toggle_gallery_favorite(gallery_id):

    # VALIDATION: Protect the operation to non-authorize person
    if "user" not in session:
        flash("You must be admin in to perform this action.", "error")
        return redirect(url_for("admin.login_page"))
    
    # CHECK: Find the specific gallery or return a 404 error
    gallery_item = Gallery.query.get_or_404(gallery_id)
    
    # VALIDATION: Ensure the gallery belongs to the logged in admin
    if gallery_item.admin_id != session['id'] : abort(403)
    
    
    
    try:
        gallery_item.is_favorite = not gallery_item.is_favorite     # Reverse the status
        db.session.commit()
        
        # user feedback
        status = 'added to favorites.' if gallery_item.is_favorite else 'removed from favorites.'
        flash(f'"{gallery_item.name}" {status}', 'success' if gallery_item.is_favorite else 'info')
    except Exception as e:
        db.session.rollback()
        flash(f'Error: {e}', 'danger')
    
    return redirect(request.referrer or url_for('gallery.index'))

@gallery_bp.route("/gallery/<int:gallery_id>/description", methods=['POST'])
def update_gallery_description(gallery_id):

    # VALIDATION: Protect the operation to non-authorize person
    if "user" not in session:
        flash("You must be admin in to perform this action.", "error")
        return redirect(url_for("admin.login_page"))
    
    # CHECK: Find the specific gallery or return a 404 error
    gallery_item = Gallery.query.get_or_404(gallery_id)
    
    # VALIDATION: Ensure the gallery belongs to the logged in admin
    if gallery_item.admin_id != session['id'] : abort(403)
    
    try:
        data = request.get_json()
        new_description = data.get('description', '').strip()[:255]
        
        gallery_item.description = new_description
        db.session.commit()
        return jsonify({"success": True, "description": gallery_item.description, "message": "Description updated successfully"})
    except Exception as e:
        db.session.rollback()
        return jsonify({"success": False, "error": str(e)}), 500



# ---------------------------------------------------------------------
# Session CRUD Operations Routes
# - typical CRUD Operations
# ---------------------------------------------------------------------
@gallery_bp.route("/session/<int:session_id>/delete", methods=['POST'])
def delete_session(session_id):
    # VALIDATION: Protect the operation to non-authorize person
    if "user" not in session:
        flash("You must be admin in to perform this action.", "error")
        return redirect(url_for("admin.login_page"))
    
    # CHECK: Find the specific session or return a 404 error
    session_obj = DBSession.query.get_or_404(session_id)
    
    # VALIDATION: Ensure the gallery belongs to the logged in admin
    if session_obj.gallery.admin_id != session['id'] : abort(403)

    
    try:
        session_obj.delete(commit=False)      # custom softdelete method
        db.session.commit()
        flash('Session has been moved to the recycle bin.', 'success')
    except Exception as e:
        db.session.rollback()
        flash(f'Error: {e}', 'danger')
    
    return redirect(request.referrer or url_for('gallery.index'))

@gallery_bp.route("/session/<int:session_id>/update", methods=['POST'])
def update_session_name(session_id):
    # VALIDATION: Protect the operation to non-authorize person
    if "user" not in session:
        flash("You must be admin in to perform this action.", "error")
        return redirect(url_for("admin.login_page"))
    
    # CHECK: Find the specific session or return a 404 error
    session_obj = DBSession.query.get_or_404(session_id)
    
    # VALIDATION: Ensure the gallery belongs to the logged in admin
    if session_obj.gallery.admin_id != session['id'] : abort(403)
    
    
    # FETCH: Get the new name from the form
    new_name = request.form.get('session_name', '').strip()
    if not new_name:
        flash('A session name is required.', 'error')
        return redirect(request.referrer)

    try:
        session_obj.name = new_name
        db.session.commit()
        flash('Session name updated successfully.', 'success')
    except Exception as e:
        db.session.rollback()
        flash(f'Error: {e}', 'danger')
    
    return redirect(request.referrer or url_for('gallery.index'))

@gallery_bp.route("/session/<int:session_id>/recover", methods=['POST'])
def recover_session(session_id):
    # VALIDATION: Protect the operation to non-authorize person
    if "user" not in session:
        flash("You must be admin in to perform this action.", "error")
        return redirect(url_for("admin.login_page"))
    
    # CHECK: Find the specific session or return a 404 error
    session_obj = DBSession.query.get_or_404(session_id)
    
    # VALIDATION: Ensure the gallery belongs to the logged in admin
    if session_obj.gallery.admin_id != session['id'] : abort(403)
    
    # EDGE CASES: Check if the gallery is already in the bin
    if getattr(session_obj.gallery, 'is_deleted', False):
        flash('You cannot recover this session because its parent Gallery is in the bin.', 'error')
        return redirect(request.referrer)
    
    
    try:
        if hasattr(session_obj, 'restore'):
            session_obj.restore(commit=False)   # Softdeletes Method
        else:
            session_obj.is_deleted = False      # Manual
        
        db.session.commit()
        flash('Session has been successfully recovered.', 'success')
    except Exception as e:
        db.session.rollback()
        flash(f'Error: {e}', 'danger')
    
    return redirect(request.referrer or url_for('gallery.index'))

@gallery_bp.route("/session/<int:session_id>/force-delete", methods=['POST'])
def force_delete_session(session_id):
    # VALIDATION: Protect the operation to non-authorize person
    if "user" not in session:
        flash("You must be admin in to perform this action.", "error")
        return redirect(url_for("admin.login_page"))
    
    # CHECK: Find the specific session or return a 404 error
    session_obj = DBSession.query.get_or_404(session_id)
    
    # VALIDATION: Ensure the gallery belongs to the logged in admin
    if session_obj.gallery.admin_id != session['id'] : abort(403)

    
    try:
        db.session.delete(session_obj)
        db.session.commit()
        flash('Session has been permanently deleted.', 'success')
    except Exception as e:
        db.session.rollback()
        flash(f'Error: {e}', 'danger')
    
    return redirect(request.referrer or url_for('gallery.index'))
