use dioxus::prelude::*;
use dioxus_i18n::{prelude::*, t};

use crate::metrics::VERSION;

/// Inline client script (no hydration / WASM — pure SSR site). It:
/// - migrates any legacy `?lang=` deep link into a `lang` cookie + localStorage,
///   then strips the query from the URL (so GitHub-Pages routing is never hit);
/// - binds the `.lang-btn` toggles to write `lang` into both localStorage and a
///   cookie, then `location.reload()` — the URL stays `/`, satisfying the
///   "no URL jump" requirement while the server re-renders the chosen language.
const LANG_TOGGLE_JS: &str = r#"
(function () {
  // Migrate any legacy ?lang= deep link into cookie + localStorage, then
  // strip the query so the URL stays clean (no GitHub-Pages routing hit).
  var p = new URLSearchParams(location.search);
  var l = p.get('lang');
  if (l) {
    document.cookie = 'lang=' + l + ';path=/;max-age=31536000';
    try { localStorage.setItem('lang', l); } catch (e) {}
    history.replaceState(null, '', location.pathname);
  }
  // Use event delegation on `document` (not per-node binding): under Dioxus
  // fullstack hydration the rendered DOM nodes are rebuilt, which would drop
  // listeners attached directly to .lang-item. A delegated listener survives
  // any DOM rebuild because it lives on the stable `document` node.
  document.addEventListener('click', function (e) {
    var t = e.target.closest('.lang-item');
    if (!t) return;
    e.preventDefault();
    var lang = t.getAttribute('data-lang') || 'zh-CN';
    document.cookie = 'lang=' + lang + ';path=/;max-age=31536000';
    try { localStorage.setItem('lang', lang); } catch (err) {}
    location.reload();
  });
})();
"#;

/// Single-page doc theme — GitBook-style: fixed left sidebar + content column.
/// Rich but tasteful: deep-space panel, grouped nav, refined cards.
/// Native in-page anchor scrolling (no router, no client JS).
const CSS: &str = include_str!("site.css");

/// Fixed left sidebar with grouped navigation. Every link is a plain
/// `<a href="#sec-...">` — the browser performs the scroll natively, so
/// navigation can never be swallowed by a hydration handler (SSR-only site).
#[component]
pub fn Sidebar() -> Element {
    let i18n = i18n();
    let zh = i18n.language() == "zh-CN";

    rsx! {
        style { {CSS} }
        div { class: "starfield-bg" }
        aside { class: "sidebar",
            div { class: "sidebar-brand",
                span { class: "sb-mark", "MPS" }
                span { class: "sb-sub", "RIGID BODY" }
                span { class: "sb-ver", "v{VERSION}" }
            }
            nav { class: "sidebar-nav",
                details { class: "nav-group", open: true,
                    summary { class: "nav-group-title", { t!("nav-group-overview") } }
                    a { class: "nav-link", href: "#sec-home", { t!("nav-home") } }
                    a { class: "nav-link", href: "#sec-quickstart", { t!("nav-quickstart") } }
                    a { class: "nav-link", href: "#sec-architecture", { t!("nav-architecture") } }
                }
                details { class: "nav-group", open: true,
                    summary { class: "nav-group-title", { t!("nav-group-core") } }
                    a { class: "nav-link", href: "#sec-gravity", { t!("nav-gravity") } }
                    a { class: "nav-link", href: "#sec-integrators", { t!("nav-integrators") } }
                    a { class: "nav-link", href: "#sec-events", { t!("nav-events") } }
                    a { class: "nav-link", href: "#sec-arena", { t!("nav-arena") } }
                    a { class: "nav-link", href: "#sec-batch", { t!("nav-batch") } }
                    a { class: "nav-link", href: "#sec-voxel", { t!("nav-voxel") } }
                    a { class: "nav-link", href: "#sec-soft-body", { t!("nav-soft-body") } }
                    a { class: "nav-link", href: "#sec-character-body", { t!("nav-character-body") } }
                    a { class: "nav-link", href: "#sec-sensor-zone", { t!("nav-sensor-zone") } }
                    a { class: "nav-link", href: "#sec-vehicle-controller", { t!("nav-vehicle-controller") } }
                    a { class: "nav-link", href: "#sec-disasters", { t!("nav-disasters") } }
                    a { class: "nav-link", href: "#sec-volcano", { t!("nav-volcano") } }
                    a { class: "nav-link", href: "#sec-force-queue", { t!("nav-force-queue") } }
                    a { class: "nav-link", href: "#sec-api", { t!("nav-api") } }
                    a { class: "nav-link", href: "#sec-changelog", { t!("nav-changelog") } }
                }
                details { class: "nav-group", open: true,
                    summary { class: "nav-group-title", { t!("nav-group-cosmos") } }
                    a { class: "nav-link", href: "#sec-cosmos", { t!("nav-cosmos") } }
                    a { class: "nav-link", href: "#sec-cosmos-world", { t!("nav-cosmos-world") } }
                    a { class: "nav-link", href: "#sec-cosmos-gravity", { t!("nav-cosmos-gravity") } }
                    a { class: "nav-link", href: "#sec-cosmos-integrator", { t!("nav-cosmos-integrator") } }
                    a { class: "nav-link", href: "#sec-cosmos-orbit", { t!("nav-cosmos-orbit") } }
                    a { class: "nav-link", href: "#sec-cosmos-flight", { t!("nav-cosmos-flight") } }
                    a { class: "nav-link", href: "#sec-cosmos-arena", { t!("nav-cosmos-arena") } }
                }
                details { class: "nav-group", open: true,
                    summary { class: "nav-group-title", { t!("nav-group-formula") } }
                    a { class: "nav-link", href: "#sec-formula", { t!("nav-formula") } }
                }
                details { class: "nav-group", open: true,
                    summary { class: "nav-group-title", { t!("nav-group-jni") } }
                    a { class: "nav-link", href: "#sec-jni", { t!("nav-jni") } }
                    a { class: "nav-link", href: "#sec-debug-replay", { t!("nav-debug-replay") } }
                }
                details { class: "nav-group", open: true,
                    summary { class: "nav-group-title", { t!("nav-group-ffm") } }
                    a { class: "nav-link", href: "#sec-ffm", { t!("nav-ffm") } }
                }
            }
            div { class: "sidebar-foot",
                p { class: "sidebar-foot-note", "MPS Rigid Body · v{VERSION}" }
            }
        }
    }
}

/// Top-right circular language dropdown.
///
/// Open/close is a native `<details>`/`<summary>` (zero client JS needed for
/// the affordance itself). Each option is a real `<a href="/?lang=...">` so a
/// click always navigates even if client JS is unavailable; the inline
/// `LANG_TOGGLE_JS` (event-delegated on `document`, which survives Dioxus
/// fullstack hydration DOM rebuilds) intercepts the click, writes the choice
/// into `localStorage` + a `lang` cookie, and reloads with the URL staying
/// `/` (no visible jump). The server re-renders in the chosen language from
/// either the cookie or the legacy `?lang=` query.
#[component]
pub fn LangDropdown() -> Element {
    let i18n = i18n();
    let zh = i18n.language() == "zh-CN";

    rsx! {
        div { class: "lang-dropdown",
            details { class: "lang-menu",
                summary { class: "lang-fab", { if zh { "中" } else { "EN" } } }
                div { class: "lang-list",
                    a { class: if zh { "lang-item is-active" } else { "lang-item" }, href: "/?lang=zh-CN", "data-lang": "zh-CN",
                        span { class: "lang-check", "✓" }
                        span { "中文" }
                    }
                    a { class: if !zh { "lang-item is-active" } else { "lang-item" }, href: "/?lang=en", "data-lang": "en",
                        span { class: "lang-check", "✓" }
                        span { "English" }
                    }
                }
            }
            script { dangerous_inner_html: LANG_TOGGLE_JS }
        }
    }
}

#[component]
pub fn Footer() -> Element {
    rsx! {
        footer { class: "site-footer",
            p {
                { t!("footer-text", version: VERSION) }
                " — "
                a { href: "https://github.com/Polari-Stars-MC/rigid-body", class: "link", "GitHub" }
            }
        }
    }
}
