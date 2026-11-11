use dioxus::prelude::*;
use dioxus_fullstack::FullstackContext;
use dioxus_i18n::{prelude::*, t};
use unic_langid::LanguageIdentifier;
use unic_langid::langid;

/// Detect the requested UI language for SSR rendering.
///
/// Preference order (all server-readable, none require a URL change):
/// 1. `lang` cookie — set by the lang toggle (survives reload with the URL
///    staying `/`, which sidesteps GitHub-Pages `?lang=` routing issues).
/// 2. legacy `?lang=` query (kept for backward-compatible deep links; the
///    client script migrates it into a cookie on first load).
///
/// Falls back to `zh-CN`.
fn detect_lang() -> LanguageIdentifier {
    if let Some(ctx) = FullstackContext::current() {
        let parts = ctx.parts_mut();
        // 1) cookie
        if let Some(s) = parts
            .headers
            .get(dioxus_fullstack::http::header::COOKIE)
            .and_then(|c| c.to_str().ok())
        {
            for kv in s.split(';') {
                let mut it = kv.trim().splitn(2, '=');
                if let (Some(k), Some(v)) = (it.next(), it.next()) {
                    if k == "lang" && v == "en" {
                        return langid!("en");
                    }
                    if k == "lang" && v == "zh-CN" {
                        return langid!("zh-CN");
                    }
                }
            }
        }
        // 2) legacy URL query
        if let Some(query) = parts.uri.query() {
            for pair in query.split('&') {
                let mut it = pair.splitn(2, '=');
                if let (Some(key), Some(val)) = (it.next(), it.next()) {
                    if key == "lang" && val == "en" {
                        return langid!("en");
                    }
                    if key == "lang" && val == "zh-CN" {
                        return langid!("zh-CN");
                    }
                }
            }
        }
    }
    langid!("zh-CN")
}

use crate::layouts::{Footer, LangDropdown, Sidebar};
use crate::metrics::{
    CELESTIAL_COUNT, CORE_FFI_COUNT, FORMULA_MODULE_COUNT, GRAVITY_MODEL_COUNT, INTEGRATOR_COUNT,
    JNI_METHOD_COUNT, TEST_COUNT,
};

use crate::pages::api::Api;
use crate::pages::architecture::Architecture;
use crate::pages::arena::Arena;
use crate::pages::batch::Batch;
use crate::pages::changelog::Changelog;
use crate::pages::character_body::CharacterBody;
use crate::pages::cosmos::Cosmos;
use crate::pages::cosmos_arena::CosmosArena;
use crate::pages::cosmos_flight::CosmosFlight;
use crate::pages::cosmos_gravity::CosmosGravity;
use crate::pages::cosmos_integrator::CosmosIntegrator;
use crate::pages::cosmos_orbit::CosmosOrbit;
use crate::pages::cosmos_world::CosmosWorld;
use crate::pages::debug_replay::DebugReplay;
use crate::pages::disasters::Disasters;
use crate::pages::events::Events;
use crate::pages::ffm::Ffm;
use crate::pages::force_queue::ForceQueue;
use crate::pages::formula::Formula;
use crate::pages::gravity::Gravity;
use crate::pages::integrators::Integrators;
use crate::pages::jni::Jni;
use crate::pages::moons::Moons;
use crate::pages::quickstart::Quickstart;
use crate::pages::sensor_zone::SensorZone;
use crate::pages::soft_body::SoftBody;
use crate::pages::vehicle_controller::VehicleController;
use crate::pages::volcano::Volcano;
use crate::pages::voxel::Voxel;

/// The whole documentation, rendered as one inline page. A sticky TOC provides
/// in-page navigation via native `#sec-...` anchor scrolling — no router, no
/// client-side click interception.
pub fn Home() -> Element {
    // Initialise i18n (Fluent) for SSR rendering.
    use_init_i18n(|| {
        let lang = detect_lang();
        let fallback = if lang == langid!("en") {
            langid!("zh-CN")
        } else {
            langid!("en")
        };
        I18nConfig::new(lang)
            .with_fallback(fallback)
            .with_locale((langid!("zh-CN"), include_str!("../i18n/locales/zh-CN.ftl")))
            .with_locale((langid!("en"), include_str!("../i18n/locales/en.ftl")))
    });

    rsx! {
        Sidebar {}
        div { class: "content-col",
            LangDropdown {}
            main { class: "page-wrap",
            section { id: "sec-home", class: "doc-section doc-home",
                div { class: "hero",
                    div { class: "hero-tag", { t!("home-hero-tag") } }
                    h1 { class: "hero-title", { t!("home-hero-title") } }
                    p { class: "hero-desc",
                        { t!("home-hero-desc", rapier: "Rapier3D-f64", ffi: CORE_FFI_COUNT, jni: JNI_METHOD_COUNT, tests: TEST_COUNT, gravity: GRAVITY_MODEL_COUNT, integrators: INTEGRATOR_COUNT, modules: FORMULA_MODULE_COUNT, bodies: CELESTIAL_COUNT) }
                    }
                    div { class: "hero-actions",
                        a { href: "#sec-quickstart", class: "btn-primary", { t!("home-cta-quickstart") } }
                        a { href: "#sec-api", class: "btn-outline", { t!("home-cta-api") } }
                    }
                }

                div { class: "metric-grid",
                    div { class: "metric-card", strong { class: "num", { TEST_COUNT } } span { class: "label", { t!("home-stat-tests") } } }
                    div { class: "metric-card", strong { class: "num", "300+" } span { class: "label", { t!("home-stat-formula-fns") } } }
                    div { class: "metric-card", strong { class: "num", { FORMULA_MODULE_COUNT } } span { class: "label", { t!("home-stat-formula-modules") } } }
                    div { class: "metric-card", strong { class: "num", { CELESTIAL_COUNT } } span { class: "label", { t!("home-stat-celestial") } } }
                }

                div { class: "module-grid",
                    a { href: "#sec-architecture", class: "module-card", span { class: "idx", "01" } strong { class: "title", { t!("home-mod-core-title") } } small { class: "desc", { t!("home-mod-core-desc") } } em { class: "arrow", "↗" } }
                    a { href: "#sec-cosmos", class: "module-card", span { class: "idx", "06" } strong { class: "title", { t!("home-mod-cosmos-title") } } small { class: "desc", { t!("home-mod-cosmos-desc") } } em { class: "arrow", "↗" } }
                    a { href: "#sec-gravity", class: "module-card", span { class: "idx", "02" } strong { class: "title", { t!("home-mod-physics-title") } } small { class: "desc", { t!("home-mod-physics-desc") } } em { class: "arrow", "↗" } }
                    a { href: "#sec-formula", class: "module-card", span { class: "idx", "03" } strong { class: "title", { t!("home-mod-formula-title") } } small { class: "desc", { t!("home-mod-formula-desc") } } em { class: "arrow", "↗" } }
                    a { href: "#sec-arena", class: "module-card", span { class: "idx", "04" } strong { class: "title", { t!("home-mod-integration-title") } } small { class: "desc", { t!("home-mod-integration-desc") } } em { class: "arrow", "↗" } }
                    a { href: "#sec-api", class: "module-card", span { class: "idx", "05" } strong { class: "title", { t!("home-mod-reference-title") } } small { class: "desc", { t!("home-mod-reference-desc") } } em { class: "arrow", "↗" } }
                }
            }

            Quickstart {}
            Architecture {}
            Gravity {}
            Integrators {}
            Formula {}
            Voxel {}
            SoftBody {}
            CharacterBody {}
            SensorZone {}
            VehicleController {}
            Disasters {}
            Volcano {}
            Changelog {}
            Moons {}
            Events {}
            ForceQueue {}
            Arena {}
            Batch {}
            Cosmos {}
            CosmosWorld {}
            CosmosGravity {}
            CosmosIntegrator {}
            CosmosOrbit {}
            CosmosFlight {}
            CosmosArena {}
            DebugReplay {}
            Jni {}
            Ffm {}
            Api {}
            }
        }
        Footer {}
    }
}
