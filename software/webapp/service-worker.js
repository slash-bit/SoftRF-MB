/**
 * Service Worker for SoftRF Settings
 * Enables offline support and caching of assets
 */

const CACHE_NAME = 'softrf-v1';
const ASSETS_TO_CACHE = [
  './',
  './index.html',
  'https://unpkg.com/leaflet@1.9.4/dist/leaflet.css',
  'https://unpkg.com/leaflet@1.9.4/dist/leaflet.js',
  'https://unpkg.com/leaflet@1.9.4/dist/images/marker-icon.png',
  'https://unpkg.com/leaflet@1.9.4/dist/images/marker-shadow.png',
  'https://unpkg.com/leaflet@1.9.4/dist/images/marker-icon-2x.png'
];

/**
 * Install event - cache essential assets
 */
self.addEventListener('install', event => {
  console.log('[Service Worker] Installing...');
  event.waitUntil(
    caches.open(CACHE_NAME).then(cache => {
      console.log('[Service Worker] Caching assets');
      return cache.addAll(ASSETS_TO_CACHE).catch(err => {
        console.warn('[Service Worker] Some assets failed to cache:', err);
        // Cache what we can; some URLs might fail but that's ok
        return Promise.resolve();
      });
    }).then(() => self.skipWaiting())
  );
});

/**
 * Activate event - clean up old caches
 */
self.addEventListener('activate', event => {
  console.log('[Service Worker] Activating...');
  event.waitUntil(
    caches.keys().then(cacheNames => {
      return Promise.all(
        cacheNames.map(cacheName => {
          if (cacheName !== CACHE_NAME) {
            console.log('[Service Worker] Deleting old cache:', cacheName);
            return caches.delete(cacheName);
          }
        })
      );
    }).then(() => self.clients.claim())
  );
});

/**
 * Fetch event - serve from cache, fallback to network
 * Strategy: Cache first, then network for online resources
 */
self.addEventListener('fetch', event => {
  const { request } = event;
  const url = new URL(request.url);

  // Skip non-GET requests
  if (request.method !== 'GET') {
    return;
  }

  // For same-origin requests, use cache-first strategy
  if (url.origin === location.origin) {
    event.respondWith(
      caches.match(request).then(response => {
        if (response) {
          return response;
        }
        return fetch(request).then(response => {
          // Cache successful responses
          if (response && response.status === 200 && response.type !== 'basic') {
            const responseToCache = response.clone();
            caches.open(CACHE_NAME).then(cache => {
              cache.put(request, responseToCache);
            });
          }
          return response;
        }).catch(() => {
          // Return offline page or fallback
          console.log('[Service Worker] Fetch failed for:', request.url);
          return new Response('Offline - No cached version available', {
            status: 503,
            statusText: 'Service Unavailable',
            headers: new Headers({
              'Content-Type': 'text/plain'
            })
          });
        });
      })
    );
    return;
  }

  // For CDN/external requests (Leaflet), use network-first strategy with cache fallback
  if (url.hostname.includes('unpkg.com')) {
    event.respondWith(
      fetch(request)
        .then(response => {
          // Cache successful responses
          if (response && response.status === 200) {
            const responseToCache = response.clone();
            caches.open(CACHE_NAME).then(cache => {
              cache.put(request, responseToCache);
            });
          }
          return response;
        })
        .catch(() => {
          // Fall back to cache when offline
          return caches.match(request).then(response => {
            if (response) {
              return response;
            }
            // Return error response
            return new Response('Offline - No cached version available', {
              status: 503,
              statusText: 'Service Unavailable',
              headers: new Headers({
                'Content-Type': 'text/plain'
              })
            });
          });
        })
    );
    return;
  }
});

/**
 * Background sync for sending data when connection is restored
 */
self.addEventListener('sync', event => {
  if (event.tag === 'sync-data') {
    event.waitUntil(syncData());
  }
});

async function syncData() {
  console.log('[Service Worker] Background sync triggered');
  // Data sync can be implemented here if needed
}

/**
 * Handle push notifications
 */
self.addEventListener('push', event => {
  if (event.data) {
    const options = {
      body: event.data.text(),
      icon: 'data:image/svg+xml,<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 192 192"><rect fill="%234361ee" width="192" height="192"/><text x="50%" y="50%" font-size="120" font-weight="bold" fill="white" text-anchor="middle" dominant-baseline="central">SR</text></svg>',
      badge: 'data:image/svg+xml,<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 192 192"><rect fill="%234361ee" width="192" height="192"/><text x="50%" y="50%" font-size="120" font-weight="bold" fill="white" text-anchor="middle" dominant-baseline="central">SR</text></svg>'
    };
    self.registration.showNotification('SoftRF', options);
  }
});

console.log('[Service Worker] Loaded');
