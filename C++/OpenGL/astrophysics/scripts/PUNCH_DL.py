#!/usr/bin/env python3
"""
https://claude.ai/public/artifacts/e8dd9f1f-4343-4968-92af-9d575e7125b2
Web File Downloader by Extension
Downloads all files with specified extensions from a website.
"""

import requests
from bs4 import BeautifulSoup
from urllib.parse import urlparse
import os
import time
import argparse
import sys
from typing import Set, List

class WebFileDownloader:
    def __init__(self, base_url: str, extensions: List[str], download_dir: str = "downloads", 
                 delay: float = 1.0, max_depth: int = 3):
        """
        Initialize the web file downloader.
        
        Args:
            base_url: The base URL to start crawling from
            extensions: List of file extensions to download (e.g., ['pdf', 'jpg', 'png'])
            download_dir: Directory to save downloaded files
            delay: Delay between requests in seconds
            max_depth: Maximum depth to crawl (0 = only base page)
        """
        self.base_url = base_url.rstrip('/')
        self.domain = urlparse(base_url).netloc
        self.extensions = [ext.lower().lstrip('.') for ext in extensions]
        self.download_dir = download_dir
        self.delay = delay
        self.max_depth = max_depth
        
        # Track visited URLs and downloaded files
        self.visited_urls: Set[str] = set()
        self.downloaded_files: Set[str] = set()
        
        # Create download directory
        os.makedirs( download_dir, exist_ok=True )
        # self.download_dir.mkdir(exist_ok=True)
        
        # Setup session with headers
        self.session = requests.Session()
        self.session.headers.update({
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36'
        })
    
    def is_valid_url(self, url: str) -> bool:
        """Check if URL is valid and belongs to the same domain."""
        try:
            parsed = urlparse(url)
            return parsed.netloc == self.domain and parsed.scheme in ['http', 'https']
        except:
            return False
    
    def has_target_extension(self, url: str) -> bool:
        """Check if URL has one of the target extensions."""
        try:
            path = urlparse(url).path.lower()
            return any(path.endswith(f'.{ext}') for ext in self.extensions)
        except:
            return False
    
    def download_file(self, url: str) -> bool:
        """Download a single file."""
        try:
            
            
            print(f"Downloading: {url}")
            response = self.session.get(url, stream=True, timeout=30)
            response.raise_for_status()
            
            # Generate filename
            filename = os.path.basename(urlparse(url).path)
            if not filename or '.' not in filename:
                # Generate filename from URL
                filename = f"file_{len(self.downloaded_files)}.{self.extensions[0]}"
            
            filepath = f"{self.download_dir}/{filename}"

            if filepath in self.downloaded_files:
                return True
            if os.path.isfile( filepath ):
                self.downloaded_files.add( filepath )
                return True
            
            # Download file
            with open(filepath, 'wb') as f:
                for chunk in response.iter_content(chunk_size=8192):
                    f.write(chunk)
            
            self.downloaded_files.add( filepath )
            print(f"✓ Downloaded: {filepath}")
            return True
            
        except Exception as e:
            print(f"✗ Failed to download {url}: {e}")
            return False
    
    def extract_links(self, url: str) -> List[str]:
        """Extract all links from a webpage."""
        try:
            response = self.session.get(url, timeout=30)
            response.raise_for_status()
            
            soup = BeautifulSoup(response.content, 'html.parser')
            links = []
            
            # Find all links
            for tag in soup.find_all(['a', 'link']):
                href = tag.get('href')
                if href:
                    # print( url, href )
                    url = url.split('?')[0]
                    full_url = f"{url}/{href}"
                    if self.is_valid_url(full_url):
                        links.append(full_url)
            
            return links
            
        except Exception as e:
            print(f"✗ Failed to extract links from {url}: {e}")
            return []
    
    def crawl_page(self, url: str, depth: int = 0) -> None:
        """Crawl a single page and process all links."""
        if (depth > self.max_depth) or (url in self.visited_urls):
            return
        
        print(f"Crawling: {url} (depth: {depth})")
        
        
        # Extract all links from the page
        links = self.extract_links(url)
        
        # Process each link
        for link in links:
            if link in self.visited_urls:
                continue
            else:
                self.visited_urls.add( link )
            
            # If it's a target file, download it
            if self.has_target_extension(link):
                print( link )
                self.download_file(link)
                # exit()
            
            # If it's a page and we haven't reached max depth, crawl it
            elif not self.has_target_extension(link) and depth < self.max_depth:
                time.sleep(self.delay)
                self.crawl_page(link, depth + 1)
        
        time.sleep(self.delay)
    
    def download_all(self) -> None:
        """Start the download process."""
        print(f"Starting download from: {self.base_url}")
        print(f"Target extensions: {', '.join(self.extensions)}")
        print(f"Download directory: {self.download_dir}")
        print(f"Max depth: {self.max_depth}")
        print("-" * 50)
        
        try:
            self.crawl_page(self.base_url)
            
            print("-" * 50)
            print(f"Download complete!")
            print(f"Total files downloaded: {len(self.downloaded_files)}")
            print(f"Files saved in: {self.download_dir}")
            
        except KeyboardInterrupt:
            print("\n\nDownload interrupted by user.")
            print(f"Downloaded {len(self.downloaded_files)} files so far.")
        except Exception as e:
            print(f"An error occurred: {e}")


def main():
    parser = argparse.ArgumentParser(description='Download files with specific extensions from a website')
    parser.add_argument('url', help='Base URL to start downloading from')
    parser.add_argument('extensions', nargs='+', help='File extensions to download (e.g., pdf jpg png)')
    parser.add_argument('-d', '--dir', default='downloads', help='Download directory (default: downloads)')
    parser.add_argument('--delay', type=float, default=1.0, help='Delay between requests in seconds (default: 1.0)')
    parser.add_argument('--max-depth', type=int, default=3, help='Maximum crawl depth (default: 3)')
    
    args = parser.parse_args()
    
    # Validate URL
    if not args.url.startswith(('http://', 'https://')):
        print("Error: URL must start with http:// or https://")
        sys.exit(1)
    
    # Create downloader and start
    downloader = WebFileDownloader(
        base_url=args.url,
        extensions=args.extensions,
        download_dir=args.dir,
        delay=args.delay,
        max_depth=args.max_depth
    )
    
    downloader.download_all()


if __name__ == "__main__":
    # Example usage if run directly
    baseURL = "https://download.hao.ucar.edu/pub/punch/cme_challenge_v2/cme0/"

    polarizations = [ 
        "cme0_dcmer_<DIR>_bang_0000_tB/", 
        "cme0_dcmer_<DIR>_bang_0000_pB/", 
    ]

    directions = [
        "090E",
        "030E",
        "0000",
        "060W",
    ]

    datasets = []

    for plr in polarizations:
        for drctn in directions:
            datasets.append( f"{baseURL}{plr}".replace( "<DIR>", drctn ) )

    for dSet in datasets:
        drctry = f"{dSet}".split('/')[-2]
        # Example of direct usage
        print( f"Attempt: {dSet}" )
        downloader = WebFileDownloader(
            base_url     = dSet,
            extensions   = ['fits'],
            download_dir = f"data/{drctry}",
            delay        = 0.5,
            max_depth    = 1
        )
        downloader.download_all()  # Uncomment to run