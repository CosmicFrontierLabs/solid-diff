//! Finding what changed in a PDM backup since a given time.
//!
//! The backup is only walked for `.SLDPRT` files, at any depth, because vault
//! exports are laid out differently depending on how they were taken and
//! nothing here needs to understand the directory structure.
//!
//! Timestamps come from inside each file, not from the filesystem. Every
//! modern SLDPRT carries the standard OPC core properties:
//!
//! ```xml
//! <dc:lastModifiedBy>stric</dc:lastModifiedBy>
//! <dcterms:modified>2026-07-23T18:40:10Z</dcterms:modified>
//! ```
//!
//! Copying a backup around rewrites mtimes; this does not. Present on 1,390 of
//! the 1,536 parts in the vault export -- the 146 without it are the pre-2015
//! files that have no readable geometry either.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

/// One revision of one part.
#[derive(Clone, Debug)]
pub struct Revision {
    pub path: PathBuf,
    /// Part name with the vault's id and version suffix removed, so revisions
    /// of the same part group together.
    pub key: String,
    /// The `_vN` suffix, when the export carries one.
    pub version: u32,
    /// `dcterms:modified`, an ISO-8601 UTC instant.
    pub modified: Option<String>,
    pub author: Option<String>,
}

/// A part that changed, and what to compare it against.
pub struct Changed {
    pub key: String,
    /// Newest revision at or after the cutoff.
    pub after: Revision,
    /// Newest revision from before the cutoff, if the part existed then.
    pub before: Option<Revision>,
}

/// Strip a vault export's `__<hex>_v<N>` suffix.
///
/// `570112-99__00000371_v8.SLDPRT` names revision 8 of part `570112-99`. The
/// hex in the middle is the vault's own id and differs between revisions, so
/// it cannot be part of the key.
pub fn split_key(stem: &str) -> (String, u32) {
    if let Some(cut) = stem.rfind("__") {
        let (head, tail) = (&stem[..cut], &stem[cut + 2..]);
        if let Some((hex, ver)) = tail.rsplit_once("_v") {
            if !hex.is_empty()
                && hex.chars().all(|c| c.is_ascii_hexdigit())
                && !ver.is_empty()
                && ver.chars().all(|c| c.is_ascii_digit())
            {
                return (head.to_string(), ver.parse().unwrap_or(0));
            }
        }
    }
    (stem.to_string(), 0)
}

/// Read `dcterms:modified` and `dc:lastModifiedBy` out of a part.
pub fn read_core_props(path: &Path) -> (Option<String>, Option<String>) {
    let Ok(data) = std::fs::read(path) else {
        return (None, None);
    };
    let Ok(file) = crate::container::parse(&data) else {
        return (None, None);
    };
    for s in file.streams() {
        if !s.name.ends_with("docProps/core.xml") {
            continue;
        }
        let txt = String::from_utf8_lossy(&s.data);
        let field = |tag: &str| -> Option<String> {
            let open = format!("<{tag}>");
            let i = txt.find(&open)? + open.len();
            let rest = &txt[i..];
            let j = rest.find('<')?;
            Some(rest[..j].to_string())
        };
        return (field("dcterms:modified"), field("dc:lastModifiedBy"));
    }
    (None, None)
}

/// Every `.SLDPRT` under `root`, at any depth.
pub fn scan(root: &Path) -> Vec<Revision> {
    let mut out = Vec::new();
    let mut stack = vec![root.to_path_buf()];
    while let Some(dir) = stack.pop() {
        let Ok(entries) = std::fs::read_dir(&dir) else {
            continue;
        };
        for e in entries.flatten() {
            let p = e.path();
            if p.is_dir() {
                stack.push(p);
                continue;
            }
            if !p
                .extension()
                .is_some_and(|x| x.eq_ignore_ascii_case("SLDPRT"))
            {
                continue;
            }
            let stem = p
                .file_stem()
                .unwrap_or_default()
                .to_string_lossy()
                .to_string();
            let (key, version) = split_key(&stem);
            let (modified, author) = read_core_props(&p);
            out.push(Revision {
                path: p,
                key,
                version,
                modified,
                author,
            });
        }
    }
    out.sort_by(|a, b| a.path.cmp(&b.path));
    out
}

/// Parts whose newest revision is at or after `since`.
///
/// `since` is compared against the ISO-8601 instant as text. That sorts
/// correctly for UTC timestamps in this format and means a bare `2026-06-01`
/// works as a prefix, with no date library and no timezone guessing.
///
/// Revisions are ordered by timestamp, falling back to the `_vN` suffix where
/// a file carries no date -- which is the pre-2015 files, and they cannot be
/// rendered anyway.
pub fn changes_since(revs: &[Revision], since: &str) -> Vec<Changed> {
    let mut by_key: HashMap<&str, Vec<&Revision>> = HashMap::new();
    for r in revs {
        by_key.entry(r.key.as_str()).or_default().push(r);
    }
    let mut out = Vec::new();
    for (key, mut group) in by_key {
        group.sort_by(|a, b| {
            a.modified
                .cmp(&b.modified)
                .then_with(|| a.version.cmp(&b.version))
        });
        let Some(newest) = group.last().copied() else {
            continue;
        };
        let Some(stamp) = newest.modified.as_deref() else {
            continue; // undateable: cannot say whether it changed
        };
        if stamp < since {
            continue;
        }
        let before = group
            .iter()
            .rev()
            .find(|r| r.modified.as_deref().is_some_and(|m| m < since))
            .map(|r| (*r).clone());
        out.push(Changed {
            key: key.to_string(),
            after: newest.clone(),
            before,
        });
    }
    out.sort_by(|a, b| a.key.cmp(&b.key));
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keys_group_revisions_of_one_part() {
        assert_eq!(
            split_key("570112-99__00000371_v8"),
            ("570112-99".to_string(), 8)
        );
        assert_eq!(
            split_key("570112-99__000001d2_v1"),
            ("570112-99".to_string(), 1)
        );
        // Same part, different vault ids: the id must not reach the key or
        // revisions would never group.
        assert_eq!(
            split_key("570112-99__00000371_v8").0,
            split_key("570112-99__000001d2_v1").0
        );
    }

    #[test]
    fn names_that_do_not_fit_the_pattern_are_left_alone() {
        assert_eq!(split_key("Arm_brace"), ("Arm_brace".to_string(), 0));
        // A trailing `__something` that is not `<hex>_v<n>` is part of the name.
        assert_eq!(
            split_key("plate__revision_two"),
            ("plate__revision_two".to_string(), 0)
        );
        // Non-hex id.
        assert_eq!(
            split_key("thing__zzzz_v2"),
            ("thing__zzzz_v2".to_string(), 0)
        );
    }

    fn rev(key: &str, v: u32, when: Option<&str>) -> Revision {
        Revision {
            path: PathBuf::from(format!("{key}_v{v}.SLDPRT")),
            key: key.to_string(),
            version: v,
            modified: when.map(|s| s.to_string()),
            author: None,
        }
    }

    #[test]
    fn picks_the_newest_and_the_last_one_before_the_cutoff() {
        let revs = vec![
            rev("a", 1, Some("2026-01-01T00:00:00Z")),
            rev("a", 2, Some("2026-05-01T00:00:00Z")),
            rev("a", 3, Some("2026-07-01T00:00:00Z")),
            rev("b", 1, Some("2026-01-01T00:00:00Z")),
        ];
        let c = changes_since(&revs, "2026-06-01");
        assert_eq!(c.len(), 1, "only `a` changed after the cutoff");
        assert_eq!(c[0].key, "a");
        assert_eq!(c[0].after.version, 3);
        assert_eq!(
            c[0].before.as_ref().map(|r| r.version),
            Some(2),
            "baseline is the newest revision from *before* the cutoff, not the first"
        );
    }

    #[test]
    fn a_part_with_no_earlier_revision_has_no_baseline() {
        let revs = vec![rev("new", 1, Some("2026-07-01T00:00:00Z"))];
        let c = changes_since(&revs, "2026-06-01");
        assert_eq!(c.len(), 1);
        assert!(c[0].before.is_none(), "nothing to compare a new part with");
    }

    #[test]
    fn undateable_parts_are_skipped_rather_than_guessed_at() {
        let revs = vec![rev("old", 1, None)];
        assert!(changes_since(&revs, "2026-06-01").is_empty());
    }
}
