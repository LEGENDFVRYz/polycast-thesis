import sqlite3
import bcrypt

DB_PATH = "auth.db"

def init_auth_db():
    """Create the database and admins table if not exists."""
    with sqlite3.connect(DB_PATH) as conn:
        conn.execute("""
        CREATE TABLE IF NOT EXISTS admins (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            username TEXT UNIQUE NOT NULL,
            password_hash TEXT NOT NULL
        );
        """)
        conn.commit()


def register_admin(username, password):
    """Register a new admin account (returns True if successful)."""
    password_hash = bcrypt.hashpw(password.encode('utf-8'), bcrypt.gensalt())

    try:
        with sqlite3.connect(DB_PATH) as conn:
            conn.execute("INSERT INTO admins (username, password_hash) VALUES (?, ?)",
                         (username, password_hash))
            conn.commit()
        return True
    except sqlite3.IntegrityError:
        return False  # username already exists


def verify_admin(username, password):
    """Check if the username/password is valid."""
    with sqlite3.connect(DB_PATH) as conn:
        row = conn.execute("SELECT password_hash FROM admins WHERE username = ?", (username,)).fetchone()
        if row and bcrypt.checkpw(password.encode('utf-8'), row[0]):
            return True
        return False
